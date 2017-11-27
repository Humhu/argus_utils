from collections import namedtuple
from sortedcontainers import SortedListWithKey

TimeItem = namedtuple('TimeItem', ['time', 'data'])


def ros_time_diff(a, b):
    return (a - b).to_sec()


class TimeSeries:

    def __init__(self, diff=None):
        self._items = SortedListWithKey(key=lambda item: item.time)
        if diff is None:
            diff = lambda x, y: x - y
        self._diff = diff

    def __len__(self):
        return len(self._items)

    def insert(self, time, val):
        query = self.find_exact(time)
        if query is None:
            self._items.add(TimeItem(time, val))
        else:
            self._items[query] = TimeItem(time, val)

    def time_span(self):
        return self._diff(self._items[-1].time, self._items[0].time)

    def earliest_item(self):
        return self._items[0]

    def remove_earliest(self):
        return self._items.pop(0)

    def remove_latest(self):
        return self._items.pop(-1)

    def trim_earliest(self, tlen):
        '''Removes all but the latest len time of data'''
        while self.time_span() > tlen:
            self.remove_earliest()

    def trim_earliest_to(self, t):
        '''Removes until earliest time > t'''
        while len(self._items) > 0 and self._diff(t, self._items[0].time) > 0:
            self.remove_earliest()

    def get_data(self, time, mode):
        if mode == 'closest_before':
            return self.get_closest_before(time)
        elif mode == 'closest_after':
            return self.get_closest_after(time)
        elif mode == 'closest_either':
            return self.get_closest_either(time)
        else:
            raise ValueError('Unknown mode: %s' % mode)

    def get_closest_before(self, time):
        '''Returns the TimeItem that is closest to at or before the specified time.'''
        query = self.__find_closest_before(time)
        if query is None:
            return None
        else:
            return self._items[query]

    def get_closest_after(self, time):
        '''Returns the TimeItem that is closest to at or after the specified time.'''
        query = self.__find_closest_after(time)
        if query is None:
            return None
        else:
            return self._items[query]

    def get_closest_either(self, time):
        '''Returns the TimeItem that is closest to the specified time.'''
        query = self.__find_closest(time)
        if query is None:
            return None
        else:
            return self._items[query]

    def find_exact(self, time):
        inb = self.__find_closest_before(time)
        if inb is None:
            return None
        elif self._items[inb].time == time:
            return inb
        else:
            return None

    def __find_closest_before(self, time):
        '''Returns the index that is closest to at or before the specified time.'''

        # If we have an exact match, inb is that index+1
        # Otherwise it is the index of the next biggest
        inb = self._items.bisect_right(TimeItem(time, None))

        # Index of 0 corresponds to two cases, both of which are failures:
        # 1. empty list
        # 2. time is earlier than all items in the list
        if inb == 0:
            return None

        # Either we have exact match or we want to return the previous item
        # Both correspond to the same action here
        return inb - 1

    def __find_closest_after(self, time):
        '''Returns the index that is closest to at or after the specified time.'''

        # If we have an exact match, ina is that index-1
        # Otherwise it is the index of the next biggest
        ina = self._items.bisect_left(TimeItem(time, None))

        # We fail in two cases:
        # Index of list tail corresponds to time being after all items in the list
        # We have no items
        if ina == len(self._items) or len(self._items) == 0:
            return None

        # Index of 0 can correspond to empty list or time being before all items in the list
        # We already caught the empty case previously, so we can proceed here
        return ina

    def __find_closest(self, time):
        inb = self.__find_closest_before(time)
        ina = self.__find_closest_after(time)

        if inb is None and ina is None:
            return None
        elif inb is None:
            return ina
        elif ina is None:
            return inb

        dta = self._diff(self._items[ina].time, time)
        dtb = self._diff(time, self._items[inb].time)
        if dta < dtb:
            return ina
        else:
            return inb

    def __repr__(self):
        temp = '{0}({1})'
        return temp.format(self.__class__.__name__,
                           repr(self._items))
