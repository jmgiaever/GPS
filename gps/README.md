GPS
===
Usage example, from in a folder hierachy:

./main.py
./gps

main.py
```
from gps import gps

if __name__ == "__main__":

    gps = gps.Data()

    while True:
        if gps.new_location():
            location = gps.get_location()
```
