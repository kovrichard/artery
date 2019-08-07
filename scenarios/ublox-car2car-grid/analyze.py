#!/usr/bin/env python3

import sys, re
from statistics import mean

class node:
    _stationId = -1
    _tx = {} # generationDeltaTime : simtime
    _rxSrc = {} # eventId : (simTime, sourceStationId)
    _rxT = {} # eventId : (simTime, generationDeltaTime)
    _posX = {} # eventId : (simTime, posX)
    _posY = {} # eventId : (simTime, posY)
    def addPosX(self, eventId, simTime, value):
        self._posX[int(eventId)] = (float(simTime), float(value))
    def addPosY(self, eventId, simTime, value):
        self._posY[int(eventId)] = (float(simTime), float(value))
    def addRxStaId(self, eventId, simTime, value):
        self._rxSrc[int(eventId)] = (float(simTime), int(value))
    def addRxGenDT(self, eventId, simTime, value):
        self._rxT[int(eventId)] = (float(simTime), int(value))
    def addTxStaId(self, eventId, simTime, value):
        self._stationId = int(value)
    def addTxGenDT(self, eventId, simTime, value):
        self._tx[int(value)] = float(simTime)

    def process(self):
        # position: (simtime, x, y)
        self.pos = [(self._posX[key][0], self._posX[key][1], self._posY[key][1]) for key in (self._posX.keys() & self._posY.keys())]
        # tx: geneventTime : simtime
        self.tx = self._tx
        self.stationId = self._stationId
        # rx: (simtime, sourceStationId, generationDeltaTime)
        self.rx = [(self._rxT[key][0], self._rxSrc[key][1], self._rxT[key][1]) for key in (self._rxT.keys() & self._rxSrc.keys())]
        
class measurement:
    _nodes = {}
    _perceptions = {}
    def __init__(self, filename):
        self._filename = filename

    def init(self):
        with open(self._filename) as f:
            for line in f.readlines():
                sline = line.strip().split()
                if(len(sline) != 4 and len(sline) != 5):
                    continue
                if(sline[0] == 'vector'):
                    self._parseDefinition(*sline[1:]) # Let's assume only ETV-s are present
                elif(sline[0].isdigit()):
                    self._perceptions[int(sline[0])](*sline[1:])
                else:
                    pass

    def process(self):
        for node in self._nodes:
            self._nodes[node].process()
        nodesLut = {value.stationId: value for key, value in self._nodes.items()}
        latencies= []
        for _, node in self._nodes.items():
            for (simtime, sourceStaId, genDT) in node.rx:
                sendSimTime = nodesLut[sourceStaId].tx[genDT]
                latencies.append(simtime - sendSimTime)
        print('SUMMARY')
        print('Minimum latency: {}'.format(min(latencies)))
        print('Maximum latency: {}'.format(max(latencies)))
        print('Average latency: {}'.format(mean(latencies)))

    def _parseDefinition(self, perceptId, nodeId, description, etc):
        pId = int(perceptId)
        nId = int(re.findall('(\d+)', nodeId)[0])

        if(nId not in self._nodes):
            self._nodes[nId] = node()

        cnode = self._nodes[nId]
        if(description == 'posX:vector'):
            self._perceptions[pId] = cnode.addPosX
        elif(description == 'posY:vector'):
            self._perceptions[pId] = cnode.addPosY
        elif(description == 'reception:vector(camStationId)'):
            self._perceptions[pId] = cnode.addRxStaId
        elif(description == 'reception:vector(camGenerationDeltaTime)'):
            self._perceptions[pId] = cnode.addRxGenDT
        elif(description == 'transmission:vector(camStationId)'):
            self._perceptions[pId] = cnode.addTxStaId
        elif(description == 'transmission:vector(camGenerationDeltaTime)'):
            self._perceptions[pId] = cnode.addTxGenDT
        else:
            pass

def main(filename):
    m = measurement(filename)
    m.init()
    m.process()


if __name__ == "__main__":
    if(len(sys.argv) != 2):
        print("Wrong number of parameters. Required only one .vec filename")
        sys.exit(1)
    main(sys.argv[1])
