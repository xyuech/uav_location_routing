"""
Author: xych
Date:   2024-12-08
Usage:  Logger file
"""
import logging
from datetime import datetime
import input.config as config


def get_logger(log_name):
    # Initialize and set levels
    main_logger = logging.getLogger(log_name)
    main_logger.setLevel(logging.DEBUG)

    # Create handlers
    timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
    main_handler = logging.FileHandler(config.Path.HOME + f'/output/{log_name}_{timestamp}.log')

    # Create formatters and add to handlers
    main_formatter = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')
    main_handler.setFormatter(main_formatter)

    # Add handlers to loggers
    main_logger.addHandler(main_handler)
    return main_logger


def get_solver_logger(logger_name, iter_count):
    sol_logger = logging.getLogger(f'{logger_name}_{iter_count}')
    sol_logger.setLevel(logging.DEBUG)

    # Create handler with unique filename
    # timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
    handler = logging.FileHandler(config.Path.HOME + f'/output/{logger_name}_run_{iter_count}.log')

    formatter = logging.Formatter('%(asctime)s - %(levelname)s - %(message)s')
    handler.setFormatter(formatter)
    sol_logger.addHandler(handler)

    return sol_logger

