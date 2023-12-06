import logging


def setup_logger(name, formatter, log_file, level=logging.INFO):
    handler = logging.FileHandler(log_file)
    handler.setFormatter(formatter)

    logger = logging.getLogger(name)
    logger.setLevel(level)
    logger.addHandler(handler)

    return logger


def clear_logger(log_handle):
    logging.shutdown(log_handle)


# global variable for map logger
map_logger = None
map_logger_formatter = \
    logging.Formatter('%(asctime)s %(levelname)s [%(filename)s:%(lineno)s'
                      ' - %(funcName)s] -- %(message)s')



