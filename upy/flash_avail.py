import os

_flash_info = os.statvfs("/")
_flash_avail = int(_flash_info[0] * _flash_info[3] / 1000)
print('flash avail: {}k'.format(_flash_avail))

