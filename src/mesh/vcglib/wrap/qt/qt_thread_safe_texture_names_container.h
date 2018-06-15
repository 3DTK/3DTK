/***************************************************************************
* VCGLib                                                            o o     *
* Visual and Computer Graphics Library                            o     o   *
*                                                                _   O  _   *
* Copyright(C) 2004-2014                                           \/)\/    *
* Visual Computing Lab                                            /\/|      *
* ISTI - Italian National Research Council                           |      *
*                                                                    \      *
* All rights reserved.                                                      *
*                                                                           *
* This program is free software; you can redistribute it and/or modify      *
* it under the terms of the GNU General Public License as published by      *
* the Free Software Foundation; either version 2 of the License, or         *
* (at your option) any later version.                                       *
*                                                                           *
* This program is distributed in the hope that it will be useful,           *
* but WITHOUT ANY WARRANTY; without even the implied warranty of            *
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the             *
* GNU General Public License (http://www.gnu.org/licenses/gpl.txt)          *
* for more details.                                                         *
*                                                                           *
****************************************************************************/

#ifndef __QT_THREAD_SAFE_TEXTURE_NAMES_CONTAINER_H
#define __QT_THREAD_SAFE_TEXTURE_NAMES_CONTAINER_H

#include <vector>
#include <wrap/qt/qt_thread_safe_memory_info.h>

namespace vcg
{ 
	struct QtThreadSafeTextureNamesContainer
    {
        QtThreadSafeTextureNamesContainer()
            :_tmid(),_lock(QReadWriteLock::Recursive)
        {
            _tmid.push_back(0);
        }

        ~QtThreadSafeTextureNamesContainer()
        {
        }

        void push_back(GLuint textid)
        {
            QWriteLocker locker(&_lock);
            _tmid.push_back(textid);
        }

        GLuint remove(GLuint textid)
        {
            QWriteLocker locker(&_lock);
            std::vector<GLuint>::iterator it = std::find(_tmid.begin(),_tmid.end(),textid);
            GLuint res = 0;
            if (it != _tmid.end())
            {
                res = *it;
                _tmid.erase(it);
            }
            return res;
        }

        size_t size() const
        {
            QReadLocker locker(&_lock);
            return _tmid.size();
        }

        bool empty() const
        {
            QReadLocker locker(&_lock);
            return _tmid.empty();
        }

        GLuint operator[](size_t ii) const
        {
            QReadLocker locker(&_lock);
            return _tmid[ii];
        };

        ////WARNING! By itself when i exit from the function scope the returned vector is NOT thread-safe!!!! 
        ////BUT after i use it in another function that is thread-safe again, so it should not be problems...maybe...
        const std::vector<GLuint>& textId() const
        {
            QReadLocker locker(&_lock);
            return _tmid;
        }
    private:
        std::vector<GLuint> _tmid;
        mutable QReadWriteLock _lock;
    };
}

#endif