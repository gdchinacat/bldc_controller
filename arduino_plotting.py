#!/usr/bin/python3
import serial
import contextlib
import numpy
import pylab
import struct
import threading

HEADER = b"-=-=-=-=\r\n"
FIELD_DELIMITER = ','
FIELD_DEF_DELIMITER = ':'
DATAPOINTS = 20000  # number of datapoints on graph

pylab.interactive(True)

colors = iter('rgbcmyk')

class Field(object):

    def __init__(self, field_def, parent):
        self.name, self.fmt, self._type = field_def.split(FIELD_DEF_DELIMITER)
        self.size = struct.calcsize(self.fmt)
        self.parent = parent
        self.overflow_count = 0
        self.last_value = 0
        self.line = None  # set in create_series (need to clean this up)
        self.create_series()

    @property
    def max_value(self):
        # does not take signed into account, because rollover doesn't make sense
        return ((1 << (8 * self.size)) - 1)

    def create_series(self):
        if not self.line:
            axes, loc = self.parent.axes_loc(self.name)
            if axes:
                series = numpy.array([0] * DATAPOINTS)
                self.line, = axes.plot(self.x, series, next(colors) + ".", label=self.name)
                if loc:
                    axes.legend(loc=loc)
        else:
            series = numpy.array([0] * DATAPOINTS)
            self.line.set_ydata(series)
            self.line.set_xdata(self.x)

    @property
    def x(self):
        return self.parent.x

    def process(self, value):
        #if self.name=='odd': print("before:", value, end="")
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

        #if self.name=='odd': print("->", value)
        return value
            

class Samples(object):
    """consume and produce samples by buffering them between threads. helps not fall behind on reading the samples"""

    fields = None

    def __init__(self, uno):
        self.lock = threading.Condition()  # protects self.series between buffer and stats
        self.uno = uno
        
        self.figure = pylab.figure(figsize=(15,7))
        self.left_axes = self.figure.add_subplot(111)
        self.left_axes.set_xlabel("time (s)")
        self.left_axes.grid("on")

        self.right_axes = self.left_axes.twinx()
        pylab.draw()
        
        self.handshake()
        threading.Thread(target=self.buffer, daemon=True).start()
        
    def handshake(self):
    
        line = None
        while line != HEADER:
            try:
                line = self.uno.readline()
                if line.endswith(HEADER):
                    break
            except:
                pass
        line = self.uno.readline()
        while line.endswith(HEADER):
            line = self.uno.readline()
            
        line = line.decode('utf-8').strip()
        _fields = line.split(FIELD_DELIMITER)
        self.field_defs = _fields
        self.create_series()
        
                
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
        self.series = [list() for _ in self.fields]

    def buffer(self):
        fmt = ">" + "".join(field.fmt for field in self.fields)  # big endian
        size = sum(field.size for field in self.fields)
        try:
            while (True):
                try:
                    # todo - some sort of error check (crc)
                    b = self.uno.read(size)
                    values = struct.unpack(fmt, b)
                    #print("buffering sample:", values)
                except (ValueError, OSError):
                    raise
                except:
                    print('bad sample:', b)
                else:
                    with self.lock:
                        for value, series, field in zip(values, self.series, self.fields):
                            value = field.process(value)
                            series.append(value)
                        self.lock.notify()
        finally:
            self.uno = None
                    
    def stats(self):
        with self.lock:
            while not len(self.series[0]):
                self.lock.wait()
            
            series = self.series
            self.series = [list() for _ in series]
            
        ret = [numpy.array(_series) for _series in series]
        return ret
                    
    def animate(self):
        while(self.uno):
            value_arrays = self.stats()
            _millis = value_arrays[0] / 1000
            value_arrays = value_arrays[1:]

            #print('data points:', len(_millis), end=" ")
            #assert all(len(a) == len(_millis) for a in value_arrays), 'invalid arrays, not same length'
            #print('average time series interval (s):', numpy.diff(_millis).mean(), end=" ")
            #print()

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

@contextlib.contextmanager
def _serial(device='/dev/ttyACM0', baud=4000000):
    """contextmanager to open/close the serial device"""
    ser = serial.Serial(device, baud)
    try:
        yield ser
    finally:
        ser.close()
        
pylab.interactive(True)

with _serial() as uno:
    samples = Samples(uno)
    threading.Thread(target=samples.animate, daemon=True).start()
    pylab.show(block=True)
    samples.uno = False  # stops the animate thread
