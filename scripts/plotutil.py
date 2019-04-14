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
import itertools

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

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
                       stdout=subprocess.PIPE)
    p.check_returncode()
    data = json.loads(p.stdout)
    ## merge scalar and vector files
    for key, val in data.items():
        if key in json_data:
            json_data[key].update(val)
        else:
            json_data[key] = val


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

def plot_points(points, *args, plot=plt, **kwargs):
    x = numpy.array()
    y = numpy.array()
    for a,b in sorted(points):
        x.append(a)
        y.append(b)
    plot.plot(x,y, *args, **kwargs)


def savefig(name, *args, plot=plt, **kwargs):
    """Saves a figure in svg, png and pdf format."""
    for ext in ('.pdf', '.png', '.svg'):
        plot.savefig(name + ext, *args, **kwargs)

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


########## Class definitions #########

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
        vec.average = np.average(self.value)

    def __repr__(self):
        return "<Vector {} of {} ({} values)>".format(
            self.name, self.module, len(self.time))

    def plot(self, label=None, *args, plot=plt, **kwargs):
        if label is None:
            label = self.name
        else:
            label = self.format_label(label)
        return plot.plot(self.time, self.value, *args, label=label, **kwargs)

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

        # scalars
        scalar_dict = dict((scalar['module'], dict()) for scalar in run_json.get('scalars', []))
        for scalar in run_json.get('scalars', []):
            scalar_dict[scalar['module']][scalar['name']] = scalar['value']
        scalars = pd.DataFrame.from_dict(self.scalar_dict, orient='index')

    def __repr__(self):
        return "<Run {}/{}/{} ({} Vectors)>".format(
            self.configname, self.measurement, self.runnumber, len(self.vectors))

    def path(self):
        return "{}/{}/{}".format(
            self.configname, self.measurement, self.runnumber)


def ret_collection(f):
    return lambda *args, **kwargs: Collection(f(*args, **kwargs))


class Collection:
    def __init__(self, generator):
        self._generator = generator;
    def __iter__(self):
        return iter(self._generator)

    @ret_collection
    def filter(self, **kwargs):
        for item in self:
            if all(getattr(item, k) == v for k,v in kwargs.items()):
                yield item

    @ret_collection
    def select(self, attr, ignore_missing=False):
        for item in self:
            if ignore_missing or hasattr(item, attr):
                yield getattr(item, attr)

    @ret_collection
    def sorted(self, attr=None, key=lambda x:x, reverse=False):
        return sorted(
            self,
            key=key if attr is None else
                lambda x: key(getattr(x, attr)),
            reverse=reverse)

    def __add__(self, other):
        return Collection(itertools.chain(this, other))

    def solidify(self):
        if not isinstance(self._generator, list):
            self._generator = list(self._generator)
        return self

    def reverse(self):
        return reversed(self)

    @ret_collection
    def __reversed__(self):
        return reversed(list(self))


class Config(Collection):
    """Models an omnet++ simulation configuration, includes multiple runs."""
    def __init__(self, runs):
        self.l = list(runs)
        self.d = _create_map(runs, 'measurement')
        self.runs = self.l
        self.moduleparams = dict(set.intersection(
            *(set(r.moduleparams.items()) for r in self.runs)))
        Collection.__init__(self, self.l)

    def __getitem__(self, item):
        if isinstance(item, int):
            return self.l[item]
        else:
            return self.d[item]

    def keys(self):
        return self.d.keys()

    def sorted(self, attr=None, key=None):
        if key is None:
            key = lambda x:x
            if attr is None:
                attr = 'measurement'
        return Collection.sorted(self, attr, key)

