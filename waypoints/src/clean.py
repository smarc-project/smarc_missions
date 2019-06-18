# Christopher Iliffe Sprague
# sprague@kth.se

import json

def clean(fname, sfname=None):

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
    f = f.replace('\n', '')

    # convert to json
    f = json.loads(f)

    # save
    if sfname is not None:
        with open(sfname, 'w') as sf:
            json.dump(f, sf, sort_keys=True, indent=4)


if __name__ == "__main__":
    clean('plan.json', 'betterplan.json')