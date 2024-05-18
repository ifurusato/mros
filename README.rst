**********************************************
MROS: Operating System for the MR01 Mars Rover
**********************************************

**MROS** provides the core functionality of the *MR01 Mars Rover*, a Raspberry
Pi-based robot OS written in Python 3.

.. figure:: https://service.robots.org.nz/wiki/attach/MR01/MR01-2024-05-18.png
   :width: 560px
   :align: center
   :height: 450px
   :alt: The MR01 Robot (Phase 1)

   A 3D model of the MR01 Robot.

|

The *MROS* library provides essential support designed as the basis of a
`Behaviour-Based Systems (BBS) <https://en.wikipedia.org/wiki/Behavior-based_robotics>`_.
This library is relatively "low-level" and, in theory, could be used for any Python 3 
based robot.

The basic function is for sensors to act as "Publishers" in a "Publish-Subscribe" model,
firing event-laden messages onto an asynchronous message bus. Subscribers to the bus can
filter which event types they are interested in. The flow of messages are thus filtered
through the Subscribers, who pass on to an Arbitrator messages they have consumed. Once all
Subscribers have acknowledged a message it is passed to a Garbage Collector (a specialised
Subscriber).

Each event type has a fixed priority. The Arbitrator receives this flow of events and
passes along to a Controller the highest priority event for a given clock cycle (typically
50ms/20Hz). The Controller takes the highest priority event and for that clock cycle
initiates any Behaviours registered for that event type.

For example, a Subscriber that filters on bumper events receives a message whose event
type is Event.BUMPER_PORT (the left/port side bumper has been triggered). This Subscriber
passes the Payload of its Message to the Arbitrator. Since a bumper press is a relatively
high priority event it's likely that it will be the highest priority and is therefore
passed on to the Controller.  If an avoidance Behaviour &mdash; let's call it AVOID_PORT
&mdash; has been registered with the Controller, it is called and the robot will begin
whatever the AvoidPort behaviour entails, perhaps stopping, backing up while turning
clockwise, then proceeding forward again on a new trajectory.


Features
********

* message and event handling
* an asynchronous message bus that forms the basis of a `Subsumption Architecture <https://en.wikipedia.org/wiki/Subsumption_architecture>`_ [#f1]_, with an "exactly-once' message delivery guarantee
* YAML-based configuration
* timestamped, multi-level, colorised [#f2]_ logging
* written in Python 3

.. [#f1] Uses finite state machines, an asynchronous message bus, an arbitrator and controller for task prioritisation.
.. [#f2] Colorised console output tested only on Unix/Linux operating systems.


Requirements
************

This library requires Python 3.8.5 or newer. Some portions (modules) of the KROS
code will only run on a Raspberry Pi, though KROS Core should function
independently of the various Pi libraries.

KROS requires installation of a number of dependencies (support libraries),
which should be automatically installed via pip3 when you installed kros-core::


Status
******

Early days. The Phase 0 hardware is largely complete and migration and conversion
of the `KROS-Core <https://github.com/ifurusato/kros-core/tree/main>`_ is being used
as the basis of MROS.

.. note::

   This project is currently in a very preliminary state.

   The project is being exposed publicly so that those interested can follow its progress.


Support & Liability
*******************

This project comes with no promise of support or acceptance of liability. Use at
your own risk.


Copyright & License
*******************

All contents (including software, documentation and images) Copyright 2020-2021
by Murray Altheim. All rights reserved.

Software and documentation are distributed under the MIT License, see LICENSE
file included with project.

