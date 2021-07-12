#!/usr/bin/env python

from setuptools import setup

setup(name="mcp22xx",
      version="0.3",
      description="Python interface for the MCP22xx USB-I2C/SPI bridges",
      author="Christof Ruess",
      author_email="chris@hobbyelektronik.org",
      url="https://hobbyelektronik.org/w/index.php/MCP-USB-Bridge",
      packages=["mcp22xx"],
      install_requires=["hidapi>=0.7.99"]
      )
