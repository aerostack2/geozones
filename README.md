# geofencing

Geofencing node for Aerostack2.

Params:

- config_file (path): Polygons that define geofences are defined here. (IMPORTANT) Remember that the order in which points are given defines how the polygon is built, this means that n point will be conected to n+1 point and so on. Last point will be connected to the first point. 

- mode (gps, cartesian): Defines whether geofencing is applied to cartessian coordinates or gps coordinates, in both cases, input position will come from its respective data types.
