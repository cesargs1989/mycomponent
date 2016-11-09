/*
 *    Copyright (C) 2016 by YOUR NAME HERE
 *
 *    This file is part of RoboComp
 *
 *    RoboComp is free software: you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License as published by
 *    the Free Software Foundation, either version 3 of the License, or
 *    (at your option) any later version.
 *
 *    RoboComp is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 *
 *    You should have received a copy of the GNU General Public License
 *    along with RoboComp.  If not, see <http://www.gnu.org/licenses/>.
 */
#ifndef GOTOPOINT_H
#define GOTOPOINT_H

// QT includes
#include <QtCore/QObject>

// Ice includes
#include <Ice/Ice.h>
#include <GotoPoint.h>

#include <config.h>
#include "genericworker.h"

using namespace RoboCompGotoPoint;

class GotoPointI : public QObject , public virtual RoboCompGotoPoint::GotoPoint
{
Q_OBJECT
public:
	GotoPointI( GenericWorker *_worker, QObject *parent = 0 );
	~GotoPointI();
	
	void go(const string  &nodo, const float  x, const float  y, const float  alpha, const Ice::Current&);
	void turn(const float  speed, const Ice::Current&);
	bool atTarget(const Ice::Current&);
	void stop(const Ice::Current&);

	QMutex *mutex;
private:

	GenericWorker *worker;
public slots:


};

#endif
