# CAP1293_thijs
A library for the CAP1293 touch sensor for atmega328p, ESP32, MSP430 or STM32 (or Wire.h)
(still in development, see TODO list near top of CAP1293_thijs.h for details)
most functions are written (about half remains untested), but need to be documented (first few functions have proper C++ function-doc, the rest just have one or 2 notes for when i have to make real documentation later).
example functions are about half-done, but the sensor is performing very strangely. I need to do some more investigative debugging before i can confidently say i know how the thing works.
NOTE: the makers of this IC went a little too far down the user-friendly-ness hole, and ended up making it actually very hard to understand. I am working on an easy-mode subclass, which puts a high-level logic layer between the user and the IC, but i'm quite busy these days, so it may take a while.