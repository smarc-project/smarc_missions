# Christopher Iliffe Sprague
# sprague@kth.se

import json

def clean(fname):

    # load the pretty json file
    f = open(fname, 'r').read()

    # clean
    f = f.replace(' ', '')
    f = f.replace('\\n', '')
    f = f.replace('\\"', '"')
    f = f.replace('"\\', '"')
    f = f.replace('\\', '')

    # remove
    f = f.split(',"transitions":')[0]
    f = f.split('"maneuvers":')[1]
    #f = f.replace('\n', '')

    print(f)

if __name__ == "__main__":
    clean('plan.json')