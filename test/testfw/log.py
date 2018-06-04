import logging

class MyLogRecord(logging.LogRecord):
    _slvls = {
        logging.DEBUG: '--',
        logging.INFO: 'II',
        logging.WARNING: 'WW',
        logging.ERROR: 'EE',
    }
    _colors = {
        logging.DEBUG:   '',
        logging.INFO:    '\033[1;34m',
        logging.WARNING: '\033[1;33m',
        logging.ERROR:   '\033[1;31m',
    }
    _msgcolors = {
        logging.DEBUG:   '',
        logging.INFO:    '',
        logging.WARNING: '\033[1;33m',
        logging.ERROR:   '\033[1;31m',
    }
    def __init__(self, name, level, fn, line, msg, args, exc_info, func=None, sinfo=None, **kwargs):
        super().__init__(name, level, fn, line, msg, args, exc_info, func=func, sinfo=sinfo, **kwargs)
        self.slvl = self._slvls.get(level, level)
        self.color = self._colors.get(level, '')
        self.msgcolor = self._msgcolors.get(level, '')
        self.crst = '\033[1;m'

