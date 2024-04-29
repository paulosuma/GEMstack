import sys
import os
import timeit

sys.path.append(os.getcwd()) # to import GEMstack from top level directory

from GEMstack.onboard.planning.road_properties import RoadPropertiesHandler

road = RoadPropertiesHandler()

coords = []
coords.append((40.114675, -88.228936)) # S Wright St, next to ECE dept
coords.append((40.113527, -88.224842)) # W Stoughton St, next to Siebel center
coords.append((40.113984, -88.224009)) # N Goodwin Ave, next to Siebel center
coords.append((40.110305, -88.227873)) # near the S Wright & Green St intersection
coords.append((40.094494, -88.237436)) # St Marys Rd, next to I Hotel
coords.append((40.112706, -88.228251)) # W Springfield Ave, next to CIF
coords.append((40.116376, -88.227432)) # W University Ave, next to Beckman Institute

for coord in coords:
	t1 = timeit.default_timer()

	road.update(*coord)

	t2 = timeit.default_timer()
	print('Update Time:', '{:.3f}'.format(t2-t1), 'seconds')

	print('Name:', road.properties.name)
	print('Speed:', road.properties.max_speed)
	print('Lanes:', road.properties.lanes)
	
	t3 = timeit.default_timer()
	print('Access Time:', '{:.6f}'.format(t3-t2), 'seconds')

	print()
