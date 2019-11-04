Flocking:
	there are 4 parameters for Flocking.
	3 are for inner flocking mixing rate.
	the formular is: CharacterAvoidance*CRate + Cohesion*CRate + Match*MRate
	the default rate is 1, 1, 1.
	
	The forth parameter is for mix rate between Flocking and Persue
	the formular is Flocking*FRate + Persue.
	
Raycasting:
	Modified the old Raycasting function, This time Raycasting will check
	Left and right simultaneously with same angle. For first time, it will
	check +5 deg and -5 deg, if both angle are blocked, check + 10 and - 10
	deg. If one direction is not blocked, turn to that direction. if both 
	direction is not blocked, randomly choose one direction to turn and 
	avoid. it will check 0-180 deg. All the three rays are sphere cast with
	0.35f as it's radius.
	
	In pathFollowing, The mixrate is 1 for seek, 0.7 for wall avoidance and
	0.6 for flocking.
	