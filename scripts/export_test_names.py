#!/bin/python
####################################################################################################
# Exports all test names and their types to devliery package
####################################################################################################

from pathlib import Path
import os
import yaml

root_dir = Path(os.path.abspath(__file__)).parent
test_dir = root_dir.parent / "test"


def extract_test_names():
    test_types = ["feature", "compliance", "reference"]
    test_dict = {"feature": {}, "compliance": {}, "reference": {}}

    for file in os.listdir(test_dir):
        if file.endswith(".yml"):
            with open(test_dir / file, "r") as fd:
                cfg = yaml.safe_load(fd)

            for test_type in test_types:
                if test_type in cfg:
                    for test in cfg[test_type]["tests"]:
                        if type(test) == str:
                            test_name = test
                        if type(test) == dict:
                            test_name = test.key()

                        # Several tests are defined multiple times with @ suffix to distuiguish
                        # run with different generic configuration of the core. Strip @ to get
                        # rid of such duplicit items
                        test_name = test_name.strip("@")

                        test_dict[test_type][test_name] = None

    return test_dict


def write_test_names(test_dict):
    fd = open(test_dir / "test_list.txt", "w")

    fd.write("This file contains list of all available test names and test types which are \n"
             "offered by CTU CAN FD VIP. To run each test, set and 'test_type' to one of \n"
             "available test names listed below. Set 'test_name' to one of available test \n"
             "names for each test type as listed below. Note that when running compliance tests, \n"
             "you need compliance library compiled and linked to simulator via PLI. \n"
             "\n")

    for test_type, tests in test_dict.items():
        #print(test_type)
        #print(tests)
        fd.write("test_type = {}\n".format(test_type))
        fd.write("List of available tests = \n")
        for test in list(tests):
            fd.write("    {}\n".format(test))
        fd.write("\n")

    fd.close()


if __name__ == "__main__":
    test_dict = extract_test_names()
    write_test_names(test_dict)

    #print(test_dict)
