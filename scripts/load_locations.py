import csv

# Load in previously saved locations
def load_locations(filename, locations):
  print "opening file"
  with open(filename, 'ab+') as csvfile:
    reader = csv.DictReader(csvfile,
        fieldnames=fieldnames, delimiter='\t', quotechar='|', quoting=csv.QUOTE_MINIMAL)
    for row in reader:
      my_row = dict()
      my_row['name'] = row['name']
      my_row['posX'] = float(row['posX'])
      my_row['posY'] = float(row['posY'])
      my_row['posZ'] = float(row['posZ'])
      my_row['quat0'] = float(row['quat0'])
      my_row['quat1'] = float(row['quat1'])
      my_row['quat2'] = float(row['quat2'])
      my_row['quat3'] = float(row['quat3'])
      my_row['radius'] = float(row['radius'])
      my_row['type'] = row['type']
      locations[row['name']]=my_row
  print "file read"
