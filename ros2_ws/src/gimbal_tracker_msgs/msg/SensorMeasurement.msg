std_msgs/Header header
uint8 CARTESIAN=0
uint8 BEARING_RANGE=1
uint8 kind
# Cartesian: [x, y, z] meters. Bearing/range: [azimuth, elevation, range].
float64[3] z
# Row-major 3x3 measurement covariance.
float64[9] covariance
