/*
 Copyright (C) 2011 J. Coliz <maniacbug@ymail.com>

 This program is free software; you can redistribute it and/or
 modify it under the terms of the GNU General Public License
 version 2 as published by the Free Software Foundation.

 05/2014: Benoit Dumas <bntdumas@gmail.com>
                Modified to use the Qt framework.

 */

#include <stdlib.h>
#include <QRF24Network.h>
#include <Sync.h>

Sync::Sync(QRF24Network &_network, QObject *parent):
    QObject(parent),
    network(_network),
    app_data(NULL),
    internal_data(NULL),
    len(0),
    to_node(0)
{
}

void Sync::begin(quint16 _to_node)
{
    to_node = _to_node;
}

void Sync::reset()
{
    memcpy(internal_data,app_data,len);
}

void Sync::update()
{
    // Pump the network
    network.update();

    // Look for changes to the data
    quint8 message[32];
    quint8 *mptr = message;
    unsigned at = 0;
    while ( at < len )
    {
        if ( app_data && internal_data && app_data[at] != internal_data[at] )
        {
            // Compose a message with the deltas
            *mptr++ = at + 1;
            *mptr++ = app_data[at];

            // Update our internal view
            internal_data[at] = app_data[at];
        }
        ++at;
    }
    // Zero out the remainder
    while ( at++ < sizeof(message) )
        *mptr++ = 0;

    // If changes, send a message
    if ( *message )
    {
        // TODO handle the case where this has to be broken into
        // multiple messages
        QRF24NetworkHeader header(/*to node*/ to_node, /*type*/ 'S' /*Sync*/);
        network.write(header,message,sizeof(message));
    }

    // Look for messages from the network
    // Is there anything ready for us?
    if ( network.available() )
    {
        // If so, take a look at it
        QRF24NetworkHeader header;
        network.peek(header);

        switch (header.type)
        {
        case 'S':

            network.read(header,&message,sizeof(message));
            // Parse the message and update the vars
            mptr = message;
            at = 0;
            while ( mptr < message + sizeof(message) )
            {
                // A '0' in the first position means we are done
                if ( !*mptr )
                    break;
                quint8 pos = (*mptr++) - 1;
                quint8 val = *mptr++;

                app_data[pos] = val;
                internal_data[pos] = val;
            }
            break;
        default:
            // Leave other messages for the app
            break;
        };
    }
}
// vim:cin:ai:sts=2 sw=2 ft=cpp
