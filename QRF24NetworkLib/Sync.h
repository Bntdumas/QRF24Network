/*
 Copyright (C) 2011 J. Coliz <maniacbug@ymail.com>

 This program is free software; you can redistribute it and/or
 modify it under the terms of the GNU General Public License
 version 2 as published by the Free Software Foundation.

 05/2014: Benoit Dumas <bntdumas@gmail.com>
                Modified to use the Qt framework.

 */
#ifndef __SYNC_H__
#define __SYNC_H__

#include <stdlib.h>
#include <string.h>

#include <QRF24Network_config.h>

#include <QObject>

class QRF24Network;
/**
 * Synchronizes a shared set of variables between multiple nodes
 */

class Sync: public QObject
{
    Q_OBJECT

private:
  QRF24Network& network;
  quint8* app_data; /**< Application's copy of the data */
  quint8* internal_data; /**< Our copy of the data */
  size_t len; /**< Length of the data in bytes */
  quint16 to_node; /**< The other node we're syncing with */

public:
  /**
   * Constructor
   * @param _network Which network to syncrhonize over
   * @param parent the object's parent
   */
  Sync(QRF24Network& _network, QObject *parent = 0);

  /**
   * Begin the object
   *
   * @param _to_node Which node we are syncing with
   */
  void begin(quint16 _to_node);

  /**
   * Declare the shared data set
   * @param _data Location of shared data to be syncrhonized
   */
  template <class T>
  void registerMe(T& _data)
  {
    app_data = reinterpret_cast<quint8*>(&_data);
    len = sizeof(_data);
    internal_data = reinterpret_cast<quint8*>(malloc(len));
    reset();
  }

  /**
   * Reset the internal copy of the shared data set 
   */
  void reset();
  
  /**
   * Update the network and the shared data set
   */
  void update();
};

#endif // __SYNC_H__
// vim:cin:ai:sts=2 sw=2 ft=cpp
