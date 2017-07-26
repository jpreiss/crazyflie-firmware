import re
import os.path as path

files = ["F1_left.c", "F1_right.c", "F2.c"]

var_init = re.compile('t[0-9]* = ')
number_re = '(-?\ *[0-9]+\.[0-9]*(?:[Ee]\ *-?\ *[0-9]+)?)'

def process(cfile):
	out = cfile + 'processed.c'
	_, fname = path.split(cfile)
	name, ext = path.splitext(fname)
	if name.startswith('F1'):
		name = name + '.m'
	else:
		name = 'J'
	with open(cfile) as f, open(out, 'w') as outf:
		for line in f:
			line = line.lstrip()
			if name == 'J' and var_init.match(line):
				line = 'float ' + line
			line = re.sub(number_re, '\\1f', line)
			line = re.sub('A0', name, line)
			outf.write(line)

for f in files:
	process(f)
