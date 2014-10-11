import serial
import contextlib
import numpy
import pylab


@contextlib.contextmanager
def uno_serial(device='/dev/ttyACM0', baud=4000000):
    ser = serial.Serial(device, baud)
    try:
        yield ser
    finally:
        ser.close()

def stats(nfields, count=None, delimiter=','):
    """this does some error filtering since the stream isn't always perfect"""
    with uno_serial() as uno:
        while count or count is None:
            line = uno.readline()
            try:
                line = line.decode('utf-8')
            except:
                continue
            line = line.strip()
            if line and line[-1] == delimiter:
                line = line[:-1]
            parts = line.split(delimiter)
            if len(parts) == (nfields or len(parts)):
                if all(part and (part[0] == '-' or str.isdigit(part)) for part in parts):
                    ret = None
                    try:
                        ret = [int(part) for part in parts]
                    except ValueError:
                        pass
                    if ret:
                        yield ret
                    
            if count: count -= 1

def collect(batchsize):
    _stats = stats(4)
    overflow_count = 0
    while (True):
        millis = []; rpm = []; power_level=[]; interrupt_count=[]
        __interrupt_count = None # used for calculating the delta
        for _ in range(batchsize):
            _millis, _rpm, _power_level, _interrupt_count = _stats.__next__()
            _millis += 0xffff * overflow_count

            if millis and _millis < last_millis:
                # hard to tell. Was it really an overflow, or a reset? 
                if (0xffff - 20) < last_millis - _millis < (0xffff + 20):  # hardcoded
                    overflow_count += 1
                    _millis += 0xffff
                else:
                    # doesn't look like an overflow, reset it
                    millis = []; rpm = []; power_level=[]; interrupt_count=[]
                    _millis -= 0xffff * overflow_count # yuck...
                    overflow_count = 0

                __interrupt_count = None
            if __interrupt_count is not None:
                if _interrupt_count > __interrupt_count:
                    tmp = _interrupt_count
                    _interrupt_count = _interrupt_count - __interrupt_count # make this relative to the previous data point, ie gauge
                    __interrupt_count = tmp
                elif _interrupt_count < __interrupt_count: # rollover
                  tmp = _interrupt_count
                  _interrupt_count = _interrupt_count + (0xffff - __interrupt_count)
                  __interrupt_count = tmp
            else:
                __interrupt_count = _interrupt_count
                _interrupt_count = 0

            last_millis = _millis
            millis.append(_millis); rpm.append(_rpm); power_level.append(_power_level); interrupt_count.append(_interrupt_count)

        ret = numpy.array(millis), numpy.array(rpm), numpy.array(power_level), numpy.array(interrupt_count)
        yield ret

DATAPOINTS = 2000  # number of datapoints on graph
BATCHSIZE = 50  # increase to reduce graph updates and reduce cpu load

millis = numpy.array([0] * DATAPOINTS)
rpm = numpy.array([0] * DATAPOINTS)
#phase_shift = numpy.array([0] * DATAPOINTS)
#even_odd = numpy.array([0] * DATAPOINTS)
power_level = numpy.array([0] * DATAPOINTS)
interrupt_count = numpy.array([0] * DATAPOINTS)
x = numpy.array([0] * DATAPOINTS)

pylab.interactive(True)
figure = pylab.figure(figsize=(25,10))
left_axes = figure.add_subplot(111)
left_axes.set_xlabel("time (s)")
left_axes.grid("on")

right_axes = left_axes.twinx()

rpm_line, = left_axes.plot(x, rpm, "r.", label='rpm')
power_level_line, = right_axes.plot(x, power_level, "g.", label='power level')
interrupt_count_line, = right_axes.plot(x, interrupt_count, "b.", label='interrupt count')
left_axes.legend(loc="upper left")
right_axes.legend(loc="upper right")
pylab.draw()

for (_millis, _rpm, _power_level, _interrupt_count) in collect(BATCHSIZE):
    print(_millis[0], [_millis[x] - _millis[x - 1] for x in range(len(_millis))]) 
    _millis = _millis / 1000
    if _millis[0] < x[-1]:
        # time reset, reset the series
        millis = numpy.array([0] * DATAPOINTS)
        #phase_shift = numpy.array([0] * DATAPOINTS)
        #even_odd = numpy.array([0] * DATAPOINTS)
        power_level = numpy.array([0] * DATAPOINTS)
        interrupt_count = numpy.array([0] * DATAPOINTS)
        x = numpy.array([0] * DATAPOINTS)

    x = numpy.array(x[len(_millis):].tolist() + _millis.tolist())

    rpm = numpy.array(rpm[len(_rpm):].tolist() + _rpm.tolist())
    rpm_line.set_ydata(rpm); rpm_line.set_xdata(x)
    #_avg, _std = numpy.average(rpm), numpy.std(rpm)
    #print('rpm avg: ', _avg, ' std: ', _std, ' pct: ', (_std / _avg) * 100);

    #even_odd = numpy.array(even_odd[len(_even_odd):].tolist() + _even_odd.tolist())
    #even_odd_line.set_ydata(even_odd); even_odd_line.set_xdata(x)

    power_level = numpy.array(power_level[len(_power_level):].tolist() + _power_level.tolist())
    power_level_line.set_ydata(power_level); power_level_line.set_xdata(x)
    #_avg, _std = numpy.average(power_level), numpy.std(power_level)
    #print('power_level avg: ', _avg, ' std: ', _std, ' pct: ', (_std / _avg) * 100);

    _interrupt_count[0] = _interrupt_count[0] - interrupt_count[-1] # gauge, relative to previous
    interrupt_count = numpy.array(interrupt_count[len(_interrupt_count):].tolist() + _interrupt_count.tolist())
    interrupt_count_line.set_ydata(interrupt_count); interrupt_count_line.set_xdata(x)
  

    _max = rpm.max()
    _min = rpm.min()

    left_axes.axis((x.min(), x.max(), _min * 0.85, _max * 1.15))

    # todo - change this to a gauge
    _max = max(power_level.max(), interrupt_count.max())
    right_axes.axis((x.min(), x.max(), 0, _max * 1.15))

    pylab.draw()

