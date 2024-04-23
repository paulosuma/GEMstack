"""NOTE: Required packages
	overpass
"""

import overpass

r_max = 10	# max radius

def to_mps(speed):
	"""Converts the speed to m/s.
	
	For OSM graph edges, 
		maxspeed = numeric value, followed by the appropriate unit. 
		When the value is in km/h, no unit should be included. 
		Other possible units: mph (miles per hour), knots
	"""

	split = speed.split(' ')

	if len(split) == 1: # in km/h
		m = 5 / 18
	else:
		if split[1] == 'mph':
			m = 0.44704
		else: # in knots
			m = 0.514444

	return float(split[0]) * m

class RoadProperties:
	"""Finds road properties like name, speed limit, number of lanes."""

	def __init__(self):
		self.api = overpass.API()

		self.latitude = None
		self.longitude = None

		self.name = None
		self.max_speed = None
		self.lanes = None

	def get_features_within_radius(self, r):
		query = 'way(around:' + str(r) + ',' + str(self.latitude) + ',' + str(self.longitude) + ')'
		response = self.api.get(query)
		return response['features']

	def update(self, latitude, longitude):
		self.latitude = latitude
		self.longitude = longitude

		self.name = self.max_speed = self.lanes = None

		for r in range(3, r_max): # starting from 3 m since standard lane width is 12 ft
			for feature in self.get_features_within_radius(r):
				try:
					self.name = feature['properties']['name']
				except: # current feature is not a road
					continue

				try:
					self.max_speed = feature['properties']['maxspeed']
					self.lanes = feature['properties']['lanes']
				except: # no max speed/ number of lanes available
					pass
				finally:
					break

			if self.name: # found the road properties
				if self.max_speed: # is not None
					self.max_speed = to_mps(self.max_speed)
				break

	def get_road_name(self, coord):
		if self.latitude == coord[0] and self.longitude == coord[1]:
			return self.name

		self.update(*coord)
		return self.name

	def get_speed_limit(self, coord):
		if self.latitude == coord[0] and self.longitude == coord[1]:
			return self.max_speed

		self.update(*coord)
		return self.max_speed

	def get_number_of_lanes(self, coord):
		if self.latitude == coord[0] and self.longitude == coord[1]:
			return self.lanes

		self.update(*coord)
		return self.lanes
