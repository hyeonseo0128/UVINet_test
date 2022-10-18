import logging

logging.basicConfig(
    format='%(asctime)s.%(msecs)03d:%(levelname)s:[%(filename)s:%(lineno)d] > %(message)s',
    datefmt='%m/%d/%Y %H:%M:%S',
    level=logging.DEBUG
)