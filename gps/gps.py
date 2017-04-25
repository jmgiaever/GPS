try:
    import machine
    import time
    import struct
except:
    pass

'''
Parsing
'''

class NMEA(object):
    '''
    Just a class to describe the type of the received entry.
    We should consider removing this to save space.
    '''
    def __init__(self, id):
        self.__id = str(id)

class Value(object):
    ''' 
    Defines a value received from the GPS
    v, float:   floating value
    u, string:  unit (e.g m, knot etc)
    t, tuple:   time when measured.
    '''
    def __init__(self, v, u, t=(0,0,0.0)):
        self.__val = float(v)
        self.__unit = u
        self.__time = t

    def value(self):
        '''
        Returns the value
        '''
        return self.__val

    def unit(self):
        '''
        Returns the unit
        '''
        return self.__unit

    def time(self):
        '''
        Returns the measurment time.
        '''
        return self.__time

    def rad(self):
        '''
        Returns the value in radians.
        '''
        return self.__val * math.pi / 180

    def __repr__(self):
        return "%f %s" % (self.value(), self.unit())

    def __float__(self):
        return float(self.__val)

class Distance(Value):
    '''
    Value: Distance
    '''
    def __init__(self, v, u, t):
        super().__init__(v, u, t)

class Position(Value):
    '''
    Value: Position
    '''
    def __init__(self, v, u, t):
        super().__init__(v if u in ("N", "E") else -v, u, t)

    def __sub__(self, other):
        '''
        Returns a new position with the subtracted value
        '''
        return Position(self.value() - other.value())

class Speed(Value):
    '''
    Value: Speed
    '''
    def __init__(self, v, u, t):
        super().__init__(v, u, t)

    def to_kmh(self):
        '''
        Returns the speed in Km/h
        '''
        if self.unit() in ("knot", "N"):
            return Speed(self.value() * 1.85200, "kmh", self.time())
        return self

    def to_knot(self):
        '''
        Returns the speed in knot
        '''
        if self.unit() in ("kmh", "K"):
            return Speed(self.value() / 1.85200, "knot", self.time())
        return self

class HDOP(Value):
    '''
    HDOP: Horizontal dilution of precision.
    '''
    def __init__(self, v, t):
        super().__init__(v, "", t)

    def __str__(self):
        if self.value() < 1:
            return "Ide."
        elif self.value() <= 2:
            return "Exe."
        elif self.value() <= 5:
            return "Good"
        elif self.value() <= 10:
            return "Mod."
        elif self.value() <= 20:
            return "Fair"
        else:
            return "Poor"

    def __repr__(self):
        return self.__str__()

class Course(Value):
    '''
    Value: Course/direction
    '''
    def __init__(self, v, t):
        super().__init__(v, "D", t)

class Location(object):
    '''
    Location:
        Tries to read data received from the GPS.
        Sometimes the GPS will return data for som
        of the segments, e.g it may happens that
        it wont receive the GPGGA-segment every time.
    '''
    def __init__(self):
        self.__valid = False
        self.__lat = None
        self.__long = None
        self.__alt = None
        self.__height = None
        self.__speed = None
        self.__course = None
        self.__satellites = -1
        self.__hdop = None

    def __repr__(self):
        return "Satelittes: %s, Quality: %s, %s\n\tLat/long: %s/%s\n\tAlt/h: %s/%s\n\tSpeed/course: %s/%s" % (
            self.__satellites, self.__hdop, repr(self.__valid),
            self.__lat, self.__lat,
            self.__alt, self.__height,
            self.__speed, self.__course,
        )

    def set(self, msgid, segment):
        '''
        Sets data based on the segment received
        msgid, []byte:      the segment type, e.g b'$GPGGA'
        segment, []byte:    the segment itself
        '''
        data = segment.split(",")
        if msgid == b'$GPGGA':
            if len(data) >= 6 and data[5] in ("1", "2", "6"):
                t = self.__time_from_seg(data[0])
                self.__set_lat(data[1], data[2], t)
                self.__set_long(data[3], data[4], t)

                if self.__satellites < 0 and self.__seg_set(data, 6):
                    self.__satellites = int(data[6])

                if self.__seg_set(data, 7):
                    self.__set_hdop(data[7], t)

                if self.__seg_set(data, 8):
                    self.__alt = Distance(data[8], "M" if not self.__seg_set(data, 9) else data[9], t)

                if self.__seg_set(data, 10):
                    self.__height = Distance(data[10], "M" if not self.__seg_set(data, 11) else data[10], t)
        elif msgid == b'$GPGLL':
            if len(data) >= 6 and data[5] == "A":
                t = self.__time_from_seg(data[4])
                self.__set_lat(data[0], data[1], t)
                self.__set_long(data[2], data[3], t)
        elif msgid == b'$GPRMC':
            if len(data) >= 6 and data[1] == "A":
                t = self.__time_from_seg(data[0])
                self.__set_lat(data[2], data[3], t)
                self.__set_long(data[4], data[5], t)
                self.__set_speed(data[6], "N", t)
                self.__set_course(data[7], t)
        elif msgid == b'$GPVTG':
            if len(data) >= 7 and data[2] == "T":
                self.__set_speed(data[4], data[6], (0, 0, 0.0))
                self.__set_course(data[5], data[0], (0, 0, 0.0))
        elif msgid == b'$GPGSV':
            if self.__seg_set(data, 2) and data[0] == data[1]:
                self.__satellites = int(data[2])
        elif msgid == b'$GPMSS':
            pass

        # This location is only valid when it was possible to set lat/long
        self.__valid = False if self.__lat is None or self.__long is None else True

        return self.valid()

    def valid(self):
        '''
        Returns wether or not the location is valid
        '''
        return self.__valid

    def __seg_set(self, d, i):
        '''
        Set position i from segment-part.
        d, []strings:   segments in parts
        i, integer:     position to read
        '''
        return len(d) >= (i + 1) and len(d[i]) != 0 and "*" not in d[i]

    def __set_hdop(self, v, t):
        '''
        Set Horizontal dilution of precision if the HDOP
        is missing or newer than the previous measured.

        Note:   We may have several segments returning this values,
                which are (may have been) measured at different times.
        '''
        if len(v) == 0:
            return None

        if self.__hdop is None or self.__hdop.time() < t:
            self.__hdop = HDOP(v, t)

    def __set_speed(self, v, u, t):
        '''
        Set speed, but only if the speed is missing or the
        previous measured speed is older.

        Note:   We may have several segments returning this values,
                which are (may have been) measured at different times.
        '''
        if len(v) == 0:
            return None

        if self.__speed is None or self.__speed.time() < t:
            self.__speed = Speed(v, u, t)

    def __set_course(self, v, t):
        '''
        Set course, but only if the course is missing or the
        previous measured speed is older.

        Note:   We may have several segments returning this values,
                which are (may have been) measured at different times.
        '''

        if len(v) == 0:
            return None

        if self.__course is None or self.__course.time() < t:
            self.__course = Course(v, t)

    def __set_lat(self, v, d, t):
        '''
        Set latitude, but only if the latitude is missing or the
        previous measured speed is older.

        Note:   We may have several segments returning this values,
                which are (may have been) measured at different times.
        '''
        if len(v) == 0:
            return None

        if self.__lat is None or self.__lat.time() < t:
            self.__lat = Position(float(v[0:2]) + (float(v[2:]) / 60), d, t)

    def __set_long(self, v, d, t):
        '''
        Set longitude, but only if the longitude is missing or the
        previous measured speed is older.

        Note:   We may have several segments returning this values,
                which are (may have been) measured at different times.
        '''
        if len(v) == 0:
            return None

        if self.__long is None or self.__long.time() < t:
            self.__long = Position(float(v[0:3]) + (float(v[3:]) / 60), d, t)

    def __time_from_seg(self, ts):
        '''
        Get the time tuple from the time-segment
        '''
        return (int(ts[0:2]), int(ts[2:4]), float(ts[4:]))

    def longitude(self):
        '''
        Returns the longitude.
        '''
        return self.__lat.value() if self.__lat is not None else False

    def latitude(self):
        '''
        Returns the latitude
        '''
        return self.__long.value() if self.__long is not None else False

    def altitude(self):
        '''
        Returns the altitude
        '''
        return self.__alt.value() if self.__alt is not None else False

    def height(self):
        '''
        Returns the height
        '''
        return self.__height.value() if self.__height is not None else False

    def speed(self):
        '''
        Returns the speed
        '''
        return self.__speed.value() if self.__speed is not None else False

    def course(self):
        '''
        Returns the course
        '''
        return self.__course.value() if self.__course is not None else False

    def satellites(self):
        '''
        Returns the amount of satellites the GPS is connected to.
        '''
        return self.__satellites if self.__satellites >= 0 else False

    def hdop(self):
        '''
        Returns the precision
        '''
        return self.__hdop if self.__hdop is not None else False


class Data(object):
    '''
    Reads and stores data from the GPS
    '''
    def __init__(self, pins=("P3", "P4"), baud=9600):
        if machine: # Fix for pydoc
            self.__com = machine.UART(1, pins=pins, baudrate=baud)
        self.__location = None
        self.__last_update = time.time()

    def new_location(self, ttw=5):
        '''
        Waits for the GPS to return data with an 'time to wait'-interval.

        Returns True if there is a new VALID location.
        '''
        if time.time() - (self.__last_update + ttw) < 0:
            return False

        self.__data = Location()
        data = []

        while self.__com.any():
            tmp_data = self.__com.readline()

            if tmp_data[0:1] == b'$':
                self.__update(data)
                data = [tmp_data]
            elif len(data) != 0:
                data.append(tmp_data)
        else:
            self.__update(data)

        if self.__data.valid():
            print(self.__data)
        
        return self.__data.valid()

    def get_location(self):
        '''
        Returns the location-data. Should be used when new_location returns True.
        '''
        return self.__data

    def __update(self, data):
        '''
        Prepares the segments for reading.
        '''
        if len(data) == 0:
            return False

        data = b''.join(data)

        if data[len(data)-1:len(data)] not in (b'\n', b'\r'):
            print("False data: %s" % (str(data),))
            return False

        if data[len(data)-1:len(data)] != b'\n':
            data += '\n'

        if self.__data.set(data[0:6], ("%s" % (data[7:len(data)-2],))[2:-1]):
            self.__last_update = time.time()
