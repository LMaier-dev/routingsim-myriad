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
import os
import pathlib
from collections import namedtuple
import itertools
import logging
import multiprocessing

import matplotlib
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

logger = logging.getLogger()

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
    runs_ = [Run(name, run) for name, run in data.items()]
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
    get_configs()

def parse_fast(globs, directoy=None, processes=None, **kwargs):
    if processes is None:
        processes = max(os.cpu_count()-1, 1)
    directoy = pathlib.Path(load_dir if directoy is None else directoy)
    if isinstance(globs, str):
        globs = [globs]

    load_paths = map(str, itertools.chain.from_iterable(
        directoy.glob(glob) for glob in globs))
    runs = {}
    with multiprocessing.Pool(processes=processes, **kwargs) as pool:
        for run in pool.imap_unordered(_get_run_from_file, load_paths):
            if run.name in runs:
                runs[run.name].merge(run)
            else:
                runs[run.name] = run

    # todo multiple runs per measurement
    configs_dict = dict(
        (configname,  Config(configruns))
        for configname, configruns in _group_by(runs.values(), 'configname').items())

    # create object
    global configs
    configs = namedtuple('configs', [])
    for k,v in configs_dict.items():
        setattr(configs, k, v)
    return configs


##### functions for plotting ####
def plot_points(points, *args, plot=plt, **kwargs):
    x = numpy.array()
    y = numpy.array()
    for a,b in sorted(points):
        x.append(a)
        y.append(b)
    plot.plot(x,y, *args, **kwargs)


def savefig(name, *args, plot=plt, **kwargs):
    """Saves a figure in svg, png, pgf and pdf format."""
    for ext in ('.pdf', '.png', '.pgf', '.svg'):
        plot.savefig(name + ext, *args, **kwargs)

def discrete_boxplot(xcol, ycol, *args, ax=None, continious_scale=True, bridge_means=False, means_style='-', **kwargs):
    """Makes a boxplot with one box per unique value in xcol."""
    ax = ax if ax is not None else plt.axes()
    if continious_scale:
        keys = np.arange(xcol.min(), xcol.max()+1)
    else:
        keys = sorted(xcol[xcol.notna()].unique())
    ax.boxplot([ycol[xcol==i] for i in keys], labels=list(int(k) for k in keys), *args, **kwargs)
    means = pd.Series((ycol[xcol==i].mean() for i in keys), np.arange(len(keys))+1)
    if bridge_means:
        means = means[means.notna()]
    ax.plot(means.index, means, means_style)
    ax.set_ylabel(ycol.name)
    ax.set_xlabel(xcol.name)

def logscale(x=True, y=True, plot=plt):
    """Turns both axis' in the plot to logscale"""
    if x:
        plot.xscale('log')
    if y:
        plot.yscale('log')

def multibar(data, labels, xlabels, ax=None):
    """Draws a bar diagram with multiple bars per x tick."""
    ax = ax if ax is not None else plt.axes()
    assert len(data[0]) == len(xlabels)
    assert len(data) == len(labels)
    ind = np.arange(len(xlabels))
    width = 1 / len(labels) * 0.9
    offset = width/2.0*len(labels)
    for i, (series, label) in enumerate(zip(data, labels)):
        ax.bar(ind+i*width-offset, series, width, label=label, align='edge')
    ax.set_xticks(ind)
    ax.set_xticklabels(xlabels)


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


def _json_filename(path):
    return str(path)+".json"

def _get_json(path, force_new=False):
    """Returns the contets of a omnet++ result file as json"""
    jsonpath = _json_filename(path)
    if not force_new and _is_newer(path, jsonpath):
        logger.info("using existing json file %s", str(jsonpath))
        try:
            return _get_json_contents(jsonpath)
        except Exception as e:
            logger.warning("loading existing json file %s failed", jsonpath)
            logger.warning(e)
    logger.info("using scavetool to generate json file %s", jsonpath)
    p = subprocess.run(
        ['scavetool', 'x', '-o', str(jsonpath), '-F', 'JSON', str(path)])
    p.check_returncode()
    assert _is_newer(path, jsonpath)
    return _get_json_contents(jsonpath)

def _is_newer(path1, path2):
    assert os.path.isfile(path1)
    return os.path.isfile(path2) and \
        os.stat(path1).st_mtime < os.stat(path2).st_mtime

def _get_json_contents(jsonpath):
    """Returns the contets of a json file"""
    with open(jsonpath) as f:
        return json.load(f)

def _get_run_from_file(path):
    if path.endswith(".sca"):
        csvpath = path + ".csv"
        if _is_newer(path, csvpath) and _is_newer(path, csvpath+".info"):
            try:
                logger.info("reading pandas csv from file %s", csvpath)
                run = _get_run_from_json(_get_json_contents(csvpath+".info"))
                run.scalars = pd.read_csv(csvpath)
                run.scalars.rename(columns={'Unnamed: 0': 'module'}, inplace=True)
                return run
            except Exception as e:
                logger.error(e)
    run = _get_run_from_json(_get_json(path))
    if path.endswith(".sca"):
        run.write_scalars(csvpath)
    return run

def _get_run_from_json(jsondata):
    assert isinstance(jsondata, dict)
    assert len(jsondata) == 1
    ((run_name, run_json),) = jsondata.items()
    return Run(run_name, run_json)


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
        vec.average = None if len(self.value)==0 else np.average(self.value)

    def __repr__(self):
        return "<Vector {} of {} ({} values)>".format(
            self.name, self.module, len(self.time))

    def plot(self, label=None, *args, plot=plt, step=None, **kwargs):
        if label is None:
            label = self.name
        else:
            label = self.format_label(label)
        return plot.plot(self.time[::step], self.value[::step],
                         *args, label=label, **kwargs)

    def format_label(self, label):
        return label.format(run=self.run,
                            name=self.name,
                            module=self.module)
    def timeslice(self, tmin, tmax):
        imin = np.searchsorted(self.time, tmin, side='left')
        imax = np.searchsorted(self.time, tmax, side='right')
        return (self.time[imin:imax], self.value[imin:imax])

    def get(self, time, default=np.NaN):
        return self.dict.get(time, default)

    def first_time(self, key, offset=0):
        for time, val in zip(self.time[offset:], self.value[offset:]):
            if key(val):
                return time
        return np.NaN


class Run:
    """Models an omnet++ simulation run"""
    def __init__(self, name, run_json):
        # attributes
        self.name = name
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
        self.moduleparams_orig = run_json['moduleparams']

        # vectors
        self.vectors = [Vector(v, self) for v in run_json.get('vectors', [])]
        self._create_vector_attr()

        if 'scalars' in run_json:
            self.add_scalars(run_json['scalars'])
        else:
            self.scalars = pd.DataFrame()

    def add_scalars(self, scalar_list):
        # scalars
        scalar_dict = dict((scalar['module'], dict()) for scalar in scalar_list)
        for scalar in scalar_list:
            scalar_dict[scalar['module']][scalar['name']] = scalar['value']
        self.scalars = pd.DataFrame.from_dict(scalar_dict, orient='index')
        self.scalars.rename(columns={'Unnamed: 0': 'module'}, inplace=True)

    def write_scalars(self, path):
        if self.scalars.empty:
            return
        self.scalars.to_csv(path)
        with open(path+'.info', 'w') as f:
            json.dump({str(self.name): dict(
                            moduleparams=self.moduleparams_orig,
                            itervars=self.itervars,
                            attributes=self.attributes)}, f)

    def merge(self, other):
        assert not self.vectors or self.scalars.empty
        assert not self.vectors or not other.vectors
        assert self.scalars.empty or other.scalars.empty
        assert self.attributes == other.attributes
        assert self.itervars == other.itervars
        assert self.moduleparams == other.moduleparams
        if not self.vectors:
            self.vectors = other.vectors
            self._create_vector_attr()
        if self.scalars.empty:
            self.scalars = other.scalars
        return self

    def __repr__(self):
        return f"<Run {self.configname}/{self.measurement}/{self.runnumber} "\
            + f"({len(self.vectors)} vectors, {len(self.scalars)} scalars)>"

    def path(self):
        return "{}/{}/{}".format(
            self.configname, self.measurement, self.runnumber)

    def _create_vector_attr(self):
        # NOTE: this only works if vector names are unique
        for k, v in _create_map(self.vectors, 'name').items():
            setattr(self, 'v_'+k, v)


def ret_collection(f):
    return lambda *args, **kwargs: Collection(f(*args, **kwargs))


class Collection:
    def __init__(self, generator):
        self._generator = generator;
    def __iter__(self):
        return iter(self._generator)

    @ret_collection
    def filter(self, fun=None, **kwargs):
        assert fun is None or len(kwargs) == 0
        if callable(fun):
            yield from filter(fun, self)
        elif fun is not None:
            yield from (a for a, b in zip(self, fun) if b)
        else:
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

    def len(self):
        return len(self._generator)

    def __getitem__(self, key):
        self.solidify()
        return self._generator[key]

    def __bool__(self):
        self.solidify()
        return bool(self._generator)

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

def find_firsts(runs, vect_name, key, offset=0):
    return ((run, getattr(run, vect_name).first_time(key, offset))
             for run in runs)
