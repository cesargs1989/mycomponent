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
#include "gotopointI.h"

GotoPointI::GotoPointI(GenericWorker *_worker, QObject *parent) : QObject(parent)
{
	worker = _worker;
	mutex = worker->mutex;       // Shared worker mutex
}


GotoPointI::~GotoPointI()
{
}

void GotoPointI::go(const string  &nodo, const float  x, const float  y, const float  alpha, const Ice::Current&)
{
	worker->go(nodo, x, y, alpha);
}

void GotoPointI::turn(const float  speed, const Ice::Current&)
{
	worker->turn(speed);
}

bool GotoPointI::atTarget(const Ice::Current&)
{
	return worker->atTarget();
}

void GotoPointI::stop(const Ice::Current&)
{
	worker->stop();
}






