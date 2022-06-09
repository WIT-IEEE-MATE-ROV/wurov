#!/usr/bin/env python

"""

 This file is part of Enbarr.

    Enbarr is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    Enbarr is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with Enbarr.  If not, see <https://www.gnu.org/licenses/>.

"""

import json

import rospy
from autobahn.twisted.websocket import WebSocketServerProtocol, WebSocketServerFactory, listenWS
from auv.msg import mode
from auv.msg import trajectory
from twisted.internet import reactor

global trajectory_publisher
trajectory_publisher = rospy.Publisher('joystick_execution', trajectory, queue_size=3)
global mode_publisher
mode_publisher = rospy.Publisher('mode_request', mode, queue_size=3)
RATE = None


class BroadcastServerProtocol(WebSocketServerProtocol):

    def onOpen(self):
        self.factory.register(self)

    def onMessage(self, payload, isBinary):
        global trajectory_publisher, mode_publisher
        sendtraj = trajectory()
        sendmode = mode()
        sendtraj.header.stamp = rospy.Time.now()
        try:
            datadict = json.loads(payload.decode('utf-8'))
            sendtraj.angular.x = float(datadict['r'])
            sendtraj.angular.y = float(datadict['p'])
            sendtraj.angular.z = float(datadict['c'])
            sendtraj.linear.x = float(datadict['x'])
            sendtraj.linear.y = float(datadict['y'])
            sendtraj.linear.z = float(datadict['z'])

            trajectory_publisher.publish(sendtraj)
            self.sendMessage(b"Joystick executor message consumed!")

        except Exception as e:
            # If anything messes up, make sure the thrusters aren't spinning anymore
            sendtraj.angular.x = 0
            sendtraj.angular.y = 0
            sendtraj.angular.z = 0
            sendtraj.linear.x = 0
            sendtraj.linear.y = 0
            sendtraj.linear.z = 0
            sendmode.auvmode = True
            sendmode.rovmode = False
            trajectory_publisher.publish(sendtraj)
            mode_publisher.publish(sendmode)

    def connectionLost(self, reason):
        WebSocketServerProtocol.connectionLost(self, reason)
        self.factory.unregister(self)


class BroadcastServerFactory(WebSocketServerFactory):
    """
    Simple broadcast server broadcasting any message it receives to all
    currently connected clients.
    """

    def __init__(self, url):
        WebSocketServerFactory.__init__(self, url)
        self.clients = []

    def register(self, client):
        if client not in self.clients:
            print("registered client {}".format(client.peer))
            self.clients.append(client)

    def unregister(self, client):
        if client in self.clients:
            print("unregistered client {}".format(client.peer))
            self.clients.remove(client)

    def broadcast(self, msg):
        print("broadcasting message '{}' ..".format(msg))
        for c in self.clients:
            c.sendMessage(msg.encode('utf8'))
            print("message sent to {}".format(c.peer))


def mode_callback(data):
    AUVMODE = data.auvmode


def listener():
    rospy.loginfo("Started joystick executor server")
    ServerFactory = BroadcastServerFactory
    factory = ServerFactory(u"ws://127.0.0.1:9001")
    factory.protocol = BroadcastServerProtocol

    rospy.init_node('joystick_execution', anonymous=True)
    rospy.loginfo("Started joystick execution node")

    rospy.Subscriber(name='current_mode', data_class=mode, callback=mode_callback)

    listenWS(factory)
    reactor.run()

    simmode = True  # TODO: Allow command line switch of this
    host = None
    if simmode is True:
        host = u"ws://127.0.0.1:9001/"
    if simmode is False:
        host = u"ws://enbarr.local:9001/"

    sendmode = mode()
    sendmode.auvmode = True;
    sendmode.rovmode = False
    modepub.publish(sendmode)
    rospy.spin()

    #    global RATE
    #    RATE = rospy.Rate(5)


if __name__ == '__main__':
    listener()
