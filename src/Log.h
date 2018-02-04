// Author: Jeremy Faller
// Copyright (c) 2007 Digital Sweetener, Inc.
// Copyright (c) 2011, Gabor Retvari
//
// This software is distributed under the MIT license.
//   http://www.opensource.org/licenses/mit-license.php
//
// http://www.thoughtclutter.com/blog/?p=3

#if !defined(LOG_H_)
#define LOG_H_
 
#include <iostream>
 
enum DebugLevel {
        CRITICAL=1,
        INFO=2,
        LOG=3,
        DEBUG1=4,
        DEBUG2=5 
};

class LogStream {
 
public:
 
        // should only be constructed once as a global variable
        LogStream(std::ostream &str, DebugLevel level = INFO)
                : mStream(str) , mOn(false), mLevel(level) {
                mInstance = this;
        }

        LogStream& operator()(DebugLevel level){
                // if(!mInstance)
                //         // create a logger instance with default settings
                //         mInstance = new LogStream(std::cerr, INFO);
                
                mOn = level <= mLevel;
                return *mInstance;
        }

        template <class T>
        inline LogStream& operator<<(const T &inVal){
#if defined(DEBUG_LOG)
                if (mOn)
                        mStream << inVal;
#endif
                return *this;
        }

        inline LogStream& operator<<(std::ostream& (*inVal)(std::ostream&)){
#if defined(DEBUG_LOG)
                if (mOn)
                        mStream << inVal;
#endif
                return *this;
        }

        // we need this to avoid having separate template instantiations
        // for each string literal (i.e, one for each char[X] with eacx X different
        inline LogStream& operator<<(const char *inVal){
#if defined(DEBUG_LOG)
                if (mOn)
                        mStream << inVal;
#endif
                return *this;
        }

 
        DebugLevel level() const { return mLevel; }
        DebugLevel& level()      { return mLevel; }
        
        std::ostream& stream()      {return mStream; }

private:

        LogStream(LogStream const &); 
        LogStream& operator=(LogStream const &); 
        //        ~LogStream(); 

        std::ostream &mStream;
        bool mOn;
        DebugLevel mLevel;
        static LogStream *mInstance;
};
 
extern LogStream debug;

#endif // LOG_H_

/* 
   Local Variables:
   c-basic-offset: 8
   End:
*/

