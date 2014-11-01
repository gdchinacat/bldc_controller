#!/usr/bin/python3
import serial
import contextlib
import numpy
import pylab
import itertools
import functools
import traceback
import struct

HEADER = b"-=-=-=-=\r\n"
FIELD_DELIMITER = ','
FIELD_DEF_DELIMITER = ':'
DATAPOINTS = 3000  # number of datapoints on graph
BATCHSIZE = 50  # increase to reduce graph updates and reduce cpu load

pylab.interactive(True)

@contextlib.contextmanager
def uno_serial(device='/dev/ttyACM0', baud=4000000):
    ser = serial.Serial(device, baud)
    try:
        yield ser
    finally:
        ser.close()

colors = iter('rgbcmyk')

class Field(object):

    line = None
    last_value = 0

    def __init__(self, field_def, parent):
        name, fmt, _type = field_def.split(FIELD_DEF_DELIMITER)
        self.name = name
        self.fmt = fmt
        self.size = struct.calcsize(fmt)
        self._type = _type
        self.parent = parent
        self.create_series()
        self.overflow_count = 0

    @property
    def max_value(self):
        # does not take signed into account, because rollover doesn't make sense
        return ((1 << (8 * self.size)) - 1)

    def create_series(self):
        if not self.line:
            axes, loc = self.parent.axes_loc(self.name)
            if axes:
                series = numpy.array([0] * DATAPOINTS)
                self.line, = axes.plot(self.x, series, next(colors) + "", label=self.name)
                if loc:
                    axes.legend(loc=loc)
        else:
            series = numpy.array([0] * DATAPOINTS)
            self.line.set_ydata(series)
            self.line.set_xdata(self.x)

    @property
    def x(self):
        return self.parent.x

    @property
    def uno(self):
        return self.parent.uno

    def read(self):
        b = self.uno.read(self.size)
        value, = struct.unpack(self.fmt, b)
        last_value = self.last_value
        self.last_value = value

        if self._type == "count":
            # convert to delta since last
            if value < last_value:
                # overflow: 
                value += self.max_value - last_value
            else:
                value -= last_value

        if self._type == "time":
            if value < last_value:
                self.overflow_count += 1
            value += self.overflow_count * self.max_value 

        return value
            

class Fields(object):

    fields = None

    def __init__(self):
        self.figure = pylab.figure(figsize=(25,10))

        self.left_axes = self.figure.add_subplot(111)
        self.left_axes.set_xlabel("time (s)")
        self.left_axes.grid("on")

        self.right_axes = self.left_axes.twinx()
        pylab.draw()

    def axes_loc(self, field_name):
        """the (axes, legend_location) for the field name, (None, None) to not plot"""
        axes, loc = None, None
        if field_name in ('pwm_level', 'interrupt count', 'accel', 'even-odd', 'even', 'odd'):
            axes = self.right_axes
            loc = 'upper right'
        elif field_name in ('rpm', 'desired rpm', 'period', 'phase shift'):
            axes = self.left_axes
            loc = 'upper left'
        return axes, loc

    def create_series(self):
        """create or recreate the field and series"""
        self.x = numpy.array([0] * DATAPOINTS)
        if not self.fields:
            self.fields = []
            for field_def in self.field_defs:
                self.fields.append(Field(field_def, self))
        else:
            for field in self.fields:
                field.create_series()

    def stats(self, count=None):
        while count or count is None:
            fields = [field.read() for field in self.fields]
            # todo - some sort of error check (crc)
            yield fields
            if count: count -= 1

    def collect(self, batchsize):
        _stats = self.stats()
        overflow_count = 0
        while (True):
            fields = [[] for _ in self.fields]
            while(len(fields[0]) < batchsize):
                for _fields, value in zip(fields, next(_stats)):
                    _fields.append(value)
                #print([field[-1] for field in fields])
            ret = [numpy.array(field) for field in fields]
            yield ret
    
    
    def run(self):
        with uno_serial() as uno:
            self.uno = uno

            while self.fields is None:
                
                line = None
                while not line or line == HEADER:
                    try:
                        line = uno.readline()
                        if line.endswith(HEADER):
                            break
                    except:
                        pass
                line = uno.readline()
                try:
                    line = line.decode('utf-8').strip()
                    _fields = line.split(FIELD_DELIMITER)
                    self.field_defs = _fields
                    self.create_series()
                except:
                    self.fields = None

            for value_arrays in self.collect(BATCHSIZE):
                _millis = value_arrays[0] / 1000
                value_arrays = value_arrays[1:]

                #print(numpy.diff(_millis).mean())

                #print(_millis[0], [_millis[x] - _millis[x - 1] for x in range(len(_millis))]) 
                if _millis[0] < self.x[-1]:
                    self.create_series()

                self.x = numpy.append(self.x[len(_millis):], _millis)
                for (field, values) in zip(self.fields[1:], value_arrays):
                    if not field.line: continue
                    line = field.line
                    line.set_ydata(numpy.append(line.get_ydata()[len(values):], values))
                    line.set_xdata(self.x)

                    _min = min([_line.get_ydata().min() for _line in line.axes.lines])
                    _max = max([_line.get_ydata().max() for _line in line.axes.lines])
                    line.axes.axis((self.x.min(), self.x.max(), _min * 0.98, _max * 1.02))
                pylab.draw()

try:
  Fields().run()
finally:
  traceback.print_exc()
  pylab.show(block=True)


