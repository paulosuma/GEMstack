import time

from GEMstack.onboard.planning.road_properties import RoadProperties

road = RoadProperties()

coords = []
coords.append((40.114675, -88.228936)) # S Wright St, next to ECE dept
coords.append((40.113527, -88.224842)) # W Stoughton St, next to Siebel center
coords.append((40.113984, -88.224009)) # N Goodwin Ave, next to Siebel center
coords.append((40.110305, -88.227873)) # near the S Wright & Green St intersection
coords.append((40.094494, -88.237436)) # St Marys Rd, next to I Hotel
coords.append((40.112706, -88.228251)) # W Springfield Ave, next to CIF
coords.append((40.116376, -88.227432)) # W University Ave, next to Beckman Institute

for coord in coords:
	print(coord)

	t1 = time.time()
	name = road.get_road_name(coord)
	t2 = time.time()
	print('Name:', name)
	print('  Time:', '{:.3f}'.format(t2-t1), 'seconds')

	t1 = time.time()
	speed = road.get_speed_limit(coord)
	t2 = time.time()
	print('Speed:', speed)
	print('  Time:', '{:.9f}'.format(t2-t1), 'seconds')

	t1 = time.time()
	lanes = road.get_number_of_lanes(coord)
	t2 = time.time()
	print('Lanes:', lanes)
	print('  Time:', '{:.9f}'.format(t2-t1), 'seconds')

	print()
