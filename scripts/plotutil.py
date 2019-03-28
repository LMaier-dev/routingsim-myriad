#
# This program is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public License
# as published by the Free Software Foundation; either version 3
# of the License, or (at your option) any later version.
#

"""
A simple library to assist plotting omnet++ results with mathplotlib.pyplot
Makes heavy use of global state, just so you know...

Author: Jakob Thomas
"""
import subprocess
import json
import pathlib
from collections import namedtuple

import matplotlib.pyplot as plt
import numpy as np

# utility
labels = namedtuple('labels', dict(var='{run.measurement}'))

# global state
load_dir = '.'
json_data = dict()
configs = namedtuple('configs', [])
runs = list()

def load_result_file(path):
    """Loads a omnet++ result file to the global data dict"""
    p = subprocess.run(['scavetool', 'x', '-o', '-', '-F', 'JSON', str(path)],
                       capture_output=True)
    p.check_returncode()
    json_data.update(json.loads(p.stdout))


def clear():
    """Resets the global state"""
    global json_data
    json_data = dict()
    global configs
    configs = namedtuple('configs', [])
    global runs
    runs = list()


def load_results_from_dir(dir_=None, glob='*.vec'):
    """Loads all files matching the glob from the dir to the global data dict."""
    if dir_ is None:
        dir_ = load_dir
    for f in pathlib.Path(dir_).glob(glob):
        load_result_file(f)


def get_runs(data=None):
    if data is None:
        data = json_data
    runs_ = [Run(r) for r in data.values()]
    if data is json_data:
        global runs
        runs = runs_
    return runs_

def get_configs(data=None):
    runs = get_runs(data)
    if data is None:
        data = json_data
    # todo multiple runs per measurement
    configs_dict = dict(
        (configname,  Config(configruns))
        for configname, configruns in _group_by(runs, 'configname').items())

    # create object
    configs_ = namedtuple('configs', [])
    for k,v in configs_dict.items():
        setattr(configs_, k, v)
    if data is json_data:
        global configs
        configs = configs_

    return configs_

def parse():
    """Parses the global json data to set the configs object"""
    get_configs()

def plot_points(points, plot=plt, **kwargs):
    x = numpy.array()
    y = numpy.array()
    for a,b in sorted(points):
        x.append(a)
        y.append(b)
    plot.plot(x,y, **kwargs)

##### helper functions ####
def _group_by(list_, attr_):
    ret = {}
    for item in list_:
        key = getattr(item, attr_)
        if key not in ret:
            ret[key] = list()
        ret[key].append(item)
    return ret

def _create_map(list_, attr_):
    return dict((getattr(item, attr_), item) for item in list_)



####### Class definitions ######
class Vector:
    """Models an omnet++ result vector"""
    def __init__(self, vector_json, run=None):
        vec = self
        vec.run = run
        vec.time = np.array(vector_json['time'])
        vec.value = np.array(vector_json['value'])
        vec.name = vector_json['name']
        vec.module = vector_json['module']
        vec.json = vector_json
        vec.dict = dict(zip(vec.time, vec.value))

    def __repr__(self):
        return "<Vector {} of {} ({} values)>".format(
            self.name, self.module, len(self.time))

    def plot(self, label=None, plot=plt, **kwargs):
        if label is None:
            label = self.name
        else:
            label = self.format_label(label)
        return plot.plot(self.time, self.value, label=label, **kwargs)

    def format_label(self, label):
        return label.format(run=self.run,
                            name=self.name,
                            module=self.module)

class Run:
    """Models an omnet++ simulation run"""
    def __init__(self, run_json):
        # attributes
        self.attributes = run_json['attributes']
        for k,v in self.attributes.items():
            setattr(self, k, v)
        # itervars
        self.itervars = run_json['itervars']
        for k,v in self.itervars.items():
            setattr(self, 'it_'+k, v)
        # moduleparams
        self.moduleparams = {}
        for param in run_json['moduleparams']:
            self.moduleparams.update(param)

        # vectors
        self.vectors = [Vector(v, self) for v in run_json['vectors']]
        # NOTE: this only works if vector names are unique
        for k, v in _create_map(self.vectors, 'name').items():
            setattr(self, 'v_'+k, v)

    def __repr__(self):
        return "<Run {}/{}/{} ({} Vectors)>".format(
            self.configname, self.measurement, self.runnumber, len(self.vectors))

    def path(self):
        return "{}/{}/{}".format(
            self.configname, self.measurement, self.runnumber)

class Config:
    """Models an omnet++ simulation configuratio, includes multiple runs."""
    def __init__(self, runs):
        self.l = list(runs)
        self.d = _create_map(runs, 'measurement')
        self.runs = self.l
        self.moduleparams = dict(set.intersection(
            *(set(r.moduleparams.items()) for r in self.runs)))

    def __getitem__(self, item):
        if isinstance(item, int):
            return self.l[item]
        else:
            return self.d[item]

    def keys(self):
        return self.d.keys()
    def __iter__(self):
        return iter(self.l)

    def sorted(self, attr='measurement', intval=False):
        if intval:
            key = lambda a: int(getattr(a, attr))
        else:
            key = lambda a: getattr(a, attr)
        return sorted(self, key=key)

    def filter(self, attr, val):
        for run in self:
            if getattr(run, attr) == val:
                yield run
