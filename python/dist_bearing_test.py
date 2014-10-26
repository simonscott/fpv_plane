from math import *


"""
Computes the distance between two geographical distance.
The provided co-ordinates must be in decimal degrees.
The returned result is in meters.
"""
def calc_distance(lon1, lat1, lon2, lat2):

    # Convert decimal degrees to radians 
    lon1_r, lat1_r, lon2_r, lat2_r = map(radians, [lon1, lat1, lon2, lat2])

    # Haversine formula 
    dlon = lon2_r - lon1_r 
    dlat = lat2_r - lat1_r 
    a = sin(dlat/2)**2 + cos(lat1_r) * cos(lat2_r) * sin(dlon/2)**2
    c = 2 * atan2(sqrt(a), sqrt(1-a))
    dist_m = 6371 * 1000.0 * c
    return (int)(dist_m)





"""
Computes the bearing from one point to another.
The provided co-ordinates must be in decimal degrees.
The returned result is in degrees.
"""
def calc_bearing(lon1, lat1, lon2, lat2):

    # Convert decimal degrees to radians 
    lon1_r, lat1_r, lon2_r, lat2_r = map(radians, [lon1, lat1, lon2, lat2])

    bearing = atan2( sin(lon2_r-lon1_r) * cos(lat2_r),
                     cos(lat1_r) * sin(lat2_r) - sin(lat1_r) * cos(lat2_r) * cos(lon2_r-lon1_r))

    return (bearing % (2*pi)) / pi * 180.0 


home = (37.980198, -122.365215)
plane = (37.997042, -122.348821)
gmaps_dist = 2381.85

dist = calc_distance(home[1], home[0], plane[1], plane[0])
print 'Distance = ', dist, 'm'
bearing = calc_bearing(plane[1], plane[0], home[1], home[0])
print 'Bearing = ', bearing, 'deg'

