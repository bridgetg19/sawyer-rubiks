#turn 1D list to string w/out spaces
def list_to_string(l):
    return ''.join(i for i in l)



#check that the rubiks cube state has correct # of each color
def validate_string(str):
	if len(str) != 54:
		print("string isn't long enough")
		return False

	if str.count('U') != 9:
		print("not enough U's")
		return False

	if str.count('R') != 9:
		print("not enough R's")
		return False

	if str.count('F') != 9:
		print("not enough F's")
		return False

	if str.count('D') != 9:
		print("not enough D's")
		return False

	if str.count('L') != 9:
		print("not enough L's")
		return False

	if str.count('B') != 9:
		print("not enough B's")
		return False

	return True



#translate the rubiks cube state so it can go into kociemba algorithm
def translate_string(rubiks_str):
	mydict = {  ord("W"):ord("U"), 
	   			ord("Y"):ord("D"),
				ord("R"):ord("R"),
				ord("O"):ord("L"), 
				ord("G"):ord("F"), 
				ord("B"):ord("B") }

	print("original string: ", rubiks_str)

	input_string = rubiks_str.translate(mydict)

	if not validate_string(input_string):
		print("not valid string")
		exit(1)

	# print("kociemba.solve('"+ input_string + "')")

	return input_string


