/**
 * Cesium - https://github.com/AnalyticalGraphicsInc/cesium
 *
 * Copyright 2011-2017 Cesium Contributors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * Columbus View (Pat. Pend.)
 *
 * Portions licensed separately.
 * See https://github.com/AnalyticalGraphicsInc/cesium/blob/master/LICENSE.md for full licensing details.
 */
(function () {
define('Core/defined',[],function() {
    'use strict';

    /**
     * @exports defined
     *
     * @param {*} value The object.
     * @returns {Boolean} Returns true if the object is defined, returns false otherwise.
     *
     * @example
     * if (Cesium.defined(positions)) {
     *      doSomething();
     * } else {
     *      doSomethingElse();
     * }
     */
    function defined(value) {
        return value !== undefined && value !== null;
    }

    return defined;
});

define('Core/DeveloperError',[
        './defined'
    ], function(
        defined) {
    'use strict';

    /**
     * Constructs an exception object that is thrown due to a developer error, e.g., invalid argument,
     * argument out of range, etc.  This exception should only be thrown during development;
     * it usually indicates a bug in the calling code.  This exception should never be
     * caught; instead the calling code should strive not to generate it.
     * <br /><br />
     * On the other hand, a {@link RuntimeError} indicates an exception that may
     * be thrown at runtime, e.g., out of memory, that the calling code should be prepared
     * to catch.
     *
     * @alias DeveloperError
     * @constructor
     * @extends Error
     *
     * @param {String} [message] The error message for this exception.
     *
     * @see RuntimeError
     */
    function DeveloperError(message) {
        /**
         * 'DeveloperError' indicating that this exception was thrown due to a developer error.
         * @type {String}
         * @readonly
         */
        this.name = 'DeveloperError';

        /**
         * The explanation for why this exception was thrown.
         * @type {String}
         * @readonly
         */
        this.message = message;

        //Browsers such as IE don't have a stack property until you actually throw the error.
        var stack;
        try {
            throw new Error();
        } catch (e) {
            stack = e.stack;
        }

        /**
         * The stack trace of this exception, if available.
         * @type {String}
         * @readonly
         */
        this.stack = stack;
    }

    if (defined(Object.create)) {
        DeveloperError.prototype = Object.create(Error.prototype);
        DeveloperError.prototype.constructor = DeveloperError;
    }

    DeveloperError.prototype.toString = function() {
        var str = this.name + ': ' + this.message;

        if (defined(this.stack)) {
            str += '\n' + this.stack.toString();
        }

        return str;
    };

    /**
     * @private
     */
    DeveloperError.throwInstantiationError = function() {
        throw new DeveloperError('This function defines an interface and should not be called directly.');
    };

    return DeveloperError;
});

define('Core/Check',[
        './defined',
        './DeveloperError'
    ], function(
        defined,
        DeveloperError) {
    'use strict';

    /**
     * Contains functions for checking that supplied arguments are of a specified type
     * or meet specified conditions
     * @private
     */
    var Check = {};

    /**
     * Contains type checking functions, all using the typeof operator
     */
    Check.typeOf = {};

    function getUndefinedErrorMessage(name) {
        return name + ' is required, actual value was undefined';
    }

    function getFailedTypeErrorMessage(actual, expected, name) {
        return 'Expected ' + name + ' to be typeof ' + expected + ', actual typeof was ' + actual;
    }

    /**
     * Throws if test is not defined
     *
     * @param {String} name The name of the variable being tested
     * @param {*} test The value that is to be checked
     * @exception {DeveloperError} test must be defined
     */
    Check.defined = function (name, test) {
        if (!defined(test)) {
            throw new DeveloperError(getUndefinedErrorMessage(name));
        }
    };

    /**
     * Throws if test is not typeof 'function'
     *
     * @param {String} name The name of the variable being tested
     * @param {*} test The value to test
     * @exception {DeveloperError} test must be typeof 'function'
     */
    Check.typeOf.func = function (name, test) {
        if (typeof test !== 'function') {
            throw new DeveloperError(getFailedTypeErrorMessage(typeof test, 'function', name));
        }
    };

    /**
     * Throws if test is not typeof 'string'
     *
     * @param {String} name The name of the variable being tested
     * @param {*} test The value to test
     * @exception {DeveloperError} test must be typeof 'string'
     */
    Check.typeOf.string = function (name, test) {
        if (typeof test !== 'string') {
            throw new DeveloperError(getFailedTypeErrorMessage(typeof test, 'string', name));
        }
    };

    /**
     * Throws if test is not typeof 'number'
     *
     * @param {String} name The name of the variable being tested
     * @param {*} test The value to test
     * @exception {DeveloperError} test must be typeof 'number'
     */
    Check.typeOf.number = function (name, test) {
        if (typeof test !== 'number') {
            throw new DeveloperError(getFailedTypeErrorMessage(typeof test, 'number', name));
        }
    };

    /**
     * Throws if test is not typeof 'number' and less than limit
     *
     * @param {String} name The name of the variable being tested
     * @param {*} test The value to test
     * @param {Number} limit The limit value to compare against
     * @exception {DeveloperError} test must be typeof 'number' and less than limit
     */
    Check.typeOf.number.lessThan = function (name, test, limit) {
        Check.typeOf.number(name, test);
        if (test >= limit) {
            throw new DeveloperError('Expected ' + name + ' to be less than ' + limit + ', actual value was ' + test);
        }
    };

    /**
     * Throws if test is not typeof 'number' and less than or equal to limit
     *
     * @param {String} name The name of the variable being tested
     * @param {*} test The value to test
     * @param {Number} limit The limit value to compare against
     * @exception {DeveloperError} test must be typeof 'number' and less than or equal to limit
     */
    Check.typeOf.number.lessThanOrEquals = function (name, test, limit) {
        Check.typeOf.number(name, test);
        if (test > limit) {
            throw new DeveloperError('Expected ' + name + ' to be less than or equal to ' + limit + ', actual value was ' + test);
        }
    };

    /**
     * Throws if test is not typeof 'number' and greater than limit
     *
     * @param {String} name The name of the variable being tested
     * @param {*} test The value to test
     * @param {Number} limit The limit value to compare against
     * @exception {DeveloperError} test must be typeof 'number' and greater than limit
     */
    Check.typeOf.number.greaterThan = function (name, test, limit) {
        Check.typeOf.number(name, test);
        if (test <= limit) {
            throw new DeveloperError('Expected ' + name + ' to be greater than ' + limit + ', actual value was ' + test);
        }
    };

    /**
     * Throws if test is not typeof 'number' and greater than or equal to limit
     *
     * @param {String} name The name of the variable being tested
     * @param {*} test The value to test
     * @param {Number} limit The limit value to compare against
     * @exception {DeveloperError} test must be typeof 'number' and greater than or equal to limit
     */
    Check.typeOf.number.greaterThanOrEquals = function (name, test, limit) {
        Check.typeOf.number(name, test);
        if (test < limit) {
            throw new DeveloperError('Expected ' + name + ' to be greater than or equal to' + limit + ', actual value was ' + test);
        }
    };

    /**
     * Throws if test is not typeof 'object'
     *
     * @param {String} name The name of the variable being tested
     * @param {*} test The value to test
     * @exception {DeveloperError} test must be typeof 'object'
     */
    Check.typeOf.object = function (name, test) {
        if (typeof test !== 'object') {
            throw new DeveloperError(getFailedTypeErrorMessage(typeof test, 'object', name));
        }
    };

    /**
     * Throws if test is not typeof 'boolean'
     *
     * @param {String} name The name of the variable being tested
     * @param {*} test The value to test
     * @exception {DeveloperError} test must be typeof 'boolean'
     */
    Check.typeOf.bool = function (name, test) {
        if (typeof test !== 'boolean') {
            throw new DeveloperError(getFailedTypeErrorMessage(typeof test, 'boolean', name));
        }
    };

    /**
     * Throws if test1 and test2 is not typeof 'number' and not equal in value
     *
     * @param {String} name1 The name of the first variable being tested
     * @param {String} name2 The name of the second variable being tested against
     * @param {*} test1 The value to test
     * @param {*} test2 The value to test against
     * @exception {DeveloperError} test1 and test2 should be type of 'number' and be equal in value
     */
    Check.typeOf.number.equals = function (name1, name2, test1, test2) {
        Check.typeOf.number(name1, test1);
        Check.typeOf.number(name2, test2);
        if (test1 !== test2) {
            throw new DeveloperError(name1 + ' must be equal to ' + name2 + ', the actual values are ' + test1 + ' and ' + test2);
        }
    };

    return Check;
});

define('Core/freezeObject',[
        './defined'
    ], function(
        defined) {
    'use strict';

    /**
     * Freezes an object, using Object.freeze if available, otherwise returns
     * the object unchanged.  This function should be used in setup code to prevent
     * errors from completely halting JavaScript execution in legacy browsers.
     *
     * @private
     *
     * @exports freezeObject
     */
    var freezeObject = Object.freeze;
    if (!defined(freezeObject)) {
        freezeObject = function(o) {
            return o;
        };
    }

    return freezeObject;
});

define('Core/defaultValue',[
        './freezeObject'
    ], function(
        freezeObject) {
    'use strict';

    /**
     * Returns the first parameter if not undefined, otherwise the second parameter.
     * Useful for setting a default value for a parameter.
     *
     * @exports defaultValue
     *
     * @param {*} a
     * @param {*} b
     * @returns {*} Returns the first parameter if not undefined, otherwise the second parameter.
     *
     * @example
     * param = Cesium.defaultValue(param, 'default');
     */
    function defaultValue(a, b) {
        if (a !== undefined && a !== null) {
            return a;
        }
        return b;
    }

    /**
     * A frozen empty object that can be used as the default value for options passed as
     * an object literal.
     * @type {Object}
     */
    defaultValue.EMPTY_OBJECT = freezeObject({});

    return defaultValue;
});

/*
  I've wrapped Makoto Matsumoto and Takuji Nishimura's code in a namespace
  so it's better encapsulated. Now you can have multiple random number generators
  and they won't stomp all over eachother's state.

  If you want to use this as a substitute for Math.random(), use the random()
  method like so:

  var m = new MersenneTwister();
  var randomNumber = m.random();

  You can also call the other genrand_{foo}() methods on the instance.

  If you want to use a specific seed in order to get a repeatable random
  sequence, pass an integer into the constructor:

  var m = new MersenneTwister(123);

  and that will always produce the same random sequence.

  Sean McCullough (banksean@gmail.com)
*/

/*
   A C-program for MT19937, with initialization improved 2002/1/26.
   Coded by Takuji Nishimura and Makoto Matsumoto.

   Before using, initialize the state by using init_genrand(seed)
   or init_by_array(init_key, key_length).
*/
/**
@license
mersenne-twister.js - https://gist.github.com/banksean/300494

   Copyright (C) 1997 - 2002, Makoto Matsumoto and Takuji Nishimura,
   All rights reserved.

   Redistribution and use in source and binary forms, with or without
   modification, are permitted provided that the following conditions
   are met:

     1. Redistributions of source code must retain the above copyright
        notice, this list of conditions and the following disclaimer.

     2. Redistributions in binary form must reproduce the above copyright
        notice, this list of conditions and the following disclaimer in the
        documentation and/or other materials provided with the distribution.

     3. The names of its contributors may not be used to endorse or promote
        products derived from this software without specific prior written
        permission.

   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
   "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
   LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
   A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR
   CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
   EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
   PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
   PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
   LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
   NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
   SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
/*
   Any feedback is very welcome.
   http://www.math.sci.hiroshima-u.ac.jp/~m-mat/MT/emt.html
   email: m-mat @ math.sci.hiroshima-u.ac.jp (remove space)
*/
define('ThirdParty/mersenne-twister',[],function() {
var MersenneTwister = function(seed) {
  if (seed == undefined) {
    seed = new Date().getTime();
  }
  /* Period parameters */
  this.N = 624;
  this.M = 397;
  this.MATRIX_A = 0x9908b0df;   /* constant vector a */
  this.UPPER_MASK = 0x80000000; /* most significant w-r bits */
  this.LOWER_MASK = 0x7fffffff; /* least significant r bits */

  this.mt = new Array(this.N); /* the array for the state vector */
  this.mti=this.N+1; /* mti==N+1 means mt[N] is not initialized */

  this.init_genrand(seed);
}

/* initializes mt[N] with a seed */
MersenneTwister.prototype.init_genrand = function(s) {
  this.mt[0] = s >>> 0;
  for (this.mti=1; this.mti<this.N; this.mti++) {
      var s = this.mt[this.mti-1] ^ (this.mt[this.mti-1] >>> 30);
   this.mt[this.mti] = (((((s & 0xffff0000) >>> 16) * 1812433253) << 16) + (s & 0x0000ffff) * 1812433253)
  + this.mti;
      /* See Knuth TAOCP Vol2. 3rd Ed. P.106 for multiplier. */
      /* In the previous versions, MSBs of the seed affect   */
      /* only MSBs of the array mt[].                        */
      /* 2002/01/09 modified by Makoto Matsumoto             */
      this.mt[this.mti] >>>= 0;
      /* for >32 bit machines */
  }
}

/* initialize by an array with array-length */
/* init_key is the array for initializing keys */
/* key_length is its length */
/* slight change for C++, 2004/2/26 */
//MersenneTwister.prototype.init_by_array = function(init_key, key_length) {
//  var i, j, k;
//  this.init_genrand(19650218);
//  i=1; j=0;
//  k = (this.N>key_length ? this.N : key_length);
//  for (; k; k--) {
//    var s = this.mt[i-1] ^ (this.mt[i-1] >>> 30)
//    this.mt[i] = (this.mt[i] ^ (((((s & 0xffff0000) >>> 16) * 1664525) << 16) + ((s & 0x0000ffff) * 1664525)))
//      + init_key[j] + j; /* non linear */
//    this.mt[i] >>>= 0; /* for WORDSIZE > 32 machines */
//    i++; j++;
//    if (i>=this.N) { this.mt[0] = this.mt[this.N-1]; i=1; }
//    if (j>=key_length) j=0;
//  }
//  for (k=this.N-1; k; k--) {
//    var s = this.mt[i-1] ^ (this.mt[i-1] >>> 30);
//    this.mt[i] = (this.mt[i] ^ (((((s & 0xffff0000) >>> 16) * 1566083941) << 16) + (s & 0x0000ffff) * 1566083941))
//      - i; /* non linear */
//    this.mt[i] >>>= 0; /* for WORDSIZE > 32 machines */
//    i++;
//    if (i>=this.N) { this.mt[0] = this.mt[this.N-1]; i=1; }
//  }
//
//  this.mt[0] = 0x80000000; /* MSB is 1; assuring non-zero initial array */
//}

/* generates a random number on [0,0xffffffff]-interval */
MersenneTwister.prototype.genrand_int32 = function() {
  var y;
  var mag01 = new Array(0x0, this.MATRIX_A);
  /* mag01[x] = x * MATRIX_A  for x=0,1 */

  if (this.mti >= this.N) { /* generate N words at one time */
    var kk;

    if (this.mti == this.N+1)   /* if init_genrand() has not been called, */
      this.init_genrand(5489); /* a default initial seed is used */

    for (kk=0;kk<this.N-this.M;kk++) {
      y = (this.mt[kk]&this.UPPER_MASK)|(this.mt[kk+1]&this.LOWER_MASK);
      this.mt[kk] = this.mt[kk+this.M] ^ (y >>> 1) ^ mag01[y & 0x1];
    }
    for (;kk<this.N-1;kk++) {
      y = (this.mt[kk]&this.UPPER_MASK)|(this.mt[kk+1]&this.LOWER_MASK);
      this.mt[kk] = this.mt[kk+(this.M-this.N)] ^ (y >>> 1) ^ mag01[y & 0x1];
    }
    y = (this.mt[this.N-1]&this.UPPER_MASK)|(this.mt[0]&this.LOWER_MASK);
    this.mt[this.N-1] = this.mt[this.M-1] ^ (y >>> 1) ^ mag01[y & 0x1];

    this.mti = 0;
  }

  y = this.mt[this.mti++];

  /* Tempering */
  y ^= (y >>> 11);
  y ^= (y << 7) & 0x9d2c5680;
  y ^= (y << 15) & 0xefc60000;
  y ^= (y >>> 18);

  return y >>> 0;
}

/* generates a random number on [0,0x7fffffff]-interval */
//MersenneTwister.prototype.genrand_int31 = function() {
//  return (this.genrand_int32()>>>1);
//}

/* generates a random number on [0,1]-real-interval */
//MersenneTwister.prototype.genrand_real1 = function() {
//  return this.genrand_int32()*(1.0/4294967295.0);
//  /* divided by 2^32-1 */
//}

/* generates a random number on [0,1)-real-interval */
MersenneTwister.prototype.random = function() {
  return this.genrand_int32()*(1.0/4294967296.0);
  /* divided by 2^32 */
}

/* generates a random number on (0,1)-real-interval */
//MersenneTwister.prototype.genrand_real3 = function() {
//  return (this.genrand_int32() + 0.5)*(1.0/4294967296.0);
//  /* divided by 2^32 */
//}

/* generates a random number on [0,1) with 53-bit resolution*/
//MersenneTwister.prototype.genrand_res53 = function() {
//  var a=this.genrand_int32()>>>5, b=this.genrand_int32()>>>6;
//  return(a*67108864.0+b)*(1.0/9007199254740992.0);
//}

/* These real versions are due to Isaku Wada, 2002/01/09 added */

return MersenneTwister;
});

define('Core/Math',[
        '../ThirdParty/mersenne-twister',
        './Check',
        './defaultValue',
        './defined',
        './DeveloperError'
    ], function(
        MersenneTwister,
        Check,
        defaultValue,
        defined,
        DeveloperError) {
    'use strict';

    /**
     * Math functions.
     *
     * @exports CesiumMath
     * @alias Math
     */
    var CesiumMath = {};

    /**
     * 0.1
     * @type {Number}
     * @constant
     */
    CesiumMath.EPSILON1 = 0.1;

    /**
     * 0.01
     * @type {Number}
     * @constant
     */
    CesiumMath.EPSILON2 = 0.01;

    /**
     * 0.001
     * @type {Number}
     * @constant
     */
    CesiumMath.EPSILON3 = 0.001;

    /**
     * 0.0001
     * @type {Number}
     * @constant
     */
    CesiumMath.EPSILON4 = 0.0001;

    /**
     * 0.00001
     * @type {Number}
     * @constant
     */
    CesiumMath.EPSILON5 = 0.00001;

    /**
     * 0.000001
     * @type {Number}
     * @constant
     */
    CesiumMath.EPSILON6 = 0.000001;

    /**
     * 0.0000001
     * @type {Number}
     * @constant
     */
    CesiumMath.EPSILON7 = 0.0000001;

    /**
     * 0.00000001
     * @type {Number}
     * @constant
     */
    CesiumMath.EPSILON8 = 0.00000001;

    /**
     * 0.000000001
     * @type {Number}
     * @constant
     */
    CesiumMath.EPSILON9 = 0.000000001;

    /**
     * 0.0000000001
     * @type {Number}
     * @constant
     */
    CesiumMath.EPSILON10 = 0.0000000001;

    /**
     * 0.00000000001
     * @type {Number}
     * @constant
     */
    CesiumMath.EPSILON11 = 0.00000000001;

    /**
     * 0.000000000001
     * @type {Number}
     * @constant
     */
    CesiumMath.EPSILON12 = 0.000000000001;

    /**
     * 0.0000000000001
     * @type {Number}
     * @constant
     */
    CesiumMath.EPSILON13 = 0.0000000000001;

    /**
     * 0.00000000000001
     * @type {Number}
     * @constant
     */
    CesiumMath.EPSILON14 = 0.00000000000001;

    /**
     * 0.000000000000001
     * @type {Number}
     * @constant
     */
    CesiumMath.EPSILON15 = 0.000000000000001;

    /**
     * 0.0000000000000001
     * @type {Number}
     * @constant
     */
    CesiumMath.EPSILON16 = 0.0000000000000001;

    /**
     * 0.00000000000000001
     * @type {Number}
     * @constant
     */
    CesiumMath.EPSILON17 = 0.00000000000000001;

    /**
     * 0.000000000000000001
     * @type {Number}
     * @constant
     */
    CesiumMath.EPSILON18 = 0.000000000000000001;

    /**
     * 0.0000000000000000001
     * @type {Number}
     * @constant
     */
    CesiumMath.EPSILON19 = 0.0000000000000000001;

    /**
     * 0.00000000000000000001
     * @type {Number}
     * @constant
     */
    CesiumMath.EPSILON20 = 0.00000000000000000001;

    /**
     * 0.000000000000000000001
     * @type {Number}
     * @constant
     */
    CesiumMath.EPSILON21 = 0.000000000000000000001;

    /**
     * The gravitational parameter of the Earth in meters cubed
     * per second squared as defined by the WGS84 model: 3.986004418e14
     * @type {Number}
     * @constant
     */
    CesiumMath.GRAVITATIONALPARAMETER = 3.986004418e14;

    /**
     * Radius of the sun in meters: 6.955e8
     * @type {Number}
     * @constant
     */
    CesiumMath.SOLAR_RADIUS = 6.955e8;

    /**
     * The mean radius of the moon, according to the "Report of the IAU/IAG Working Group on
     * Cartographic Coordinates and Rotational Elements of the Planets and satellites: 2000",
     * Celestial Mechanics 82: 83-110, 2002.
     * @type {Number}
     * @constant
     */
    CesiumMath.LUNAR_RADIUS = 1737400.0;

    /**
     * 64 * 1024
     * @type {Number}
     * @constant
     */
    CesiumMath.SIXTY_FOUR_KILOBYTES = 64 * 1024;

    /**
     * 4 * 1024 * 1024 * 1024
     * @type {Number}
     * @constant
     */
    CesiumMath.FOUR_GIGABYTES = 4 * 1024 * 1024 * 1024;

    /**
     * Returns the sign of the value; 1 if the value is positive, -1 if the value is
     * negative, or 0 if the value is 0.
     *
     * @function
     * @param {Number} value The value to return the sign of.
     * @returns {Number} The sign of value.
     */
    CesiumMath.sign = defaultValue(Math.sign, function sign(value) {
        value = +value; // coerce to number
        if (value === 0 || value !== value) {
            // zero or NaN
            return value;
        }
        return value > 0 ? 1 : -1;
    });

    /**
     * Returns 1.0 if the given value is positive or zero, and -1.0 if it is negative.
     * This is similar to {@link CesiumMath#sign} except that returns 1.0 instead of
     * 0.0 when the input value is 0.0.
     * @param {Number} value The value to return the sign of.
     * @returns {Number} The sign of value.
     */
    CesiumMath.signNotZero = function(value) {
        return value < 0.0 ? -1.0 : 1.0;
    };

    /**
     * Converts a scalar value in the range [-1.0, 1.0] to a SNORM in the range [0, rangeMaximum]
     * @param {Number} value The scalar value in the range [-1.0, 1.0]
     * @param {Number} [rangeMaximum=255] The maximum value in the mapped range, 255 by default.
     * @returns {Number} A SNORM value, where 0 maps to -1.0 and rangeMaximum maps to 1.0.
     *
     * @see CesiumMath.fromSNorm
     */
    CesiumMath.toSNorm = function(value, rangeMaximum) {
        rangeMaximum = defaultValue(rangeMaximum, 255);
        return Math.round((CesiumMath.clamp(value, -1.0, 1.0) * 0.5 + 0.5) * rangeMaximum);
    };

    /**
     * Converts a SNORM value in the range [0, rangeMaximum] to a scalar in the range [-1.0, 1.0].
     * @param {Number} value SNORM value in the range [0, rangeMaximum]
     * @param {Number} [rangeMaximum=255] The maximum value in the SNORM range, 255 by default.
     * @returns {Number} Scalar in the range [-1.0, 1.0].
     *
     * @see CesiumMath.toSNorm
     */
    CesiumMath.fromSNorm = function(value, rangeMaximum) {
        rangeMaximum = defaultValue(rangeMaximum, 255);
        return CesiumMath.clamp(value, 0.0, rangeMaximum) / rangeMaximum * 2.0 - 1.0;
    };

    /**
     * Converts a scalar value in the range [rangeMinimum, rangeMaximum] to a scalar in the range [0.0, 1.0]
     * @param {Number} value The scalar value in the range [rangeMinimum, rangeMaximum]
     * @param {Number} rangeMinimum The minimum value in the mapped range.
     * @param {Number} rangeMaximum The maximum value in the mapped range.
     * @returns {Number} A scalar value, where rangeMinimum maps to 0.0 and rangeMaximum maps to 1.0.
     */
    CesiumMath.normalize = function(value, rangeMinimum, rangeMaximum) {
        rangeMaximum = Math.max(rangeMaximum - rangeMinimum, 0.0);
        return rangeMaximum === 0.0 ? 0.0 : CesiumMath.clamp((value - rangeMinimum) / rangeMaximum, 0.0, 1.0);
    };

    /**
     * Returns the hyperbolic sine of a number.
     * The hyperbolic sine of <em>value</em> is defined to be
     * (<em>e<sup>x</sup>&nbsp;-&nbsp;e<sup>-x</sup></em>)/2.0
     * where <i>e</i> is Euler's number, approximately 2.71828183.
     *
     * <p>Special cases:
     *   <ul>
     *     <li>If the argument is NaN, then the result is NaN.</li>
     *
     *     <li>If the argument is infinite, then the result is an infinity
     *     with the same sign as the argument.</li>
     *
     *     <li>If the argument is zero, then the result is a zero with the
     *     same sign as the argument.</li>
     *   </ul>
     *</p>
     *
     * @function
     * @param {Number} value The number whose hyperbolic sine is to be returned.
     * @returns {Number} The hyperbolic sine of <code>value</code>.
     */
    CesiumMath.sinh = defaultValue(Math.sinh, function sinh(value) {
        return (Math.exp(value) - Math.exp(-value)) / 2.0;
    });

    /**
     * Returns the hyperbolic cosine of a number.
     * The hyperbolic cosine of <strong>value</strong> is defined to be
     * (<em>e<sup>x</sup>&nbsp;+&nbsp;e<sup>-x</sup></em>)/2.0
     * where <i>e</i> is Euler's number, approximately 2.71828183.
     *
     * <p>Special cases:
     *   <ul>
     *     <li>If the argument is NaN, then the result is NaN.</li>
     *
     *     <li>If the argument is infinite, then the result is positive infinity.</li>
     *
     *     <li>If the argument is zero, then the result is 1.0.</li>
     *   </ul>
     *</p>
     *
     * @function
     * @param {Number} value The number whose hyperbolic cosine is to be returned.
     * @returns {Number} The hyperbolic cosine of <code>value</code>.
     */
    CesiumMath.cosh = defaultValue(Math.cosh, function cosh(value) {
        return (Math.exp(value) + Math.exp(-value)) / 2.0;
    });

    /**
     * Computes the linear interpolation of two values.
     *
     * @param {Number} p The start value to interpolate.
     * @param {Number} q The end value to interpolate.
     * @param {Number} time The time of interpolation generally in the range <code>[0.0, 1.0]</code>.
     * @returns {Number} The linearly interpolated value.
     *
     * @example
     * var n = Cesium.Math.lerp(0.0, 2.0, 0.5); // returns 1.0
     */
    CesiumMath.lerp = function(p, q, time) {
        return ((1.0 - time) * p) + (time * q);
    };

    /**
     * pi
     *
     * @type {Number}
     * @constant
     */
    CesiumMath.PI = Math.PI;

    /**
     * 1/pi
     *
     * @type {Number}
     * @constant
     */
    CesiumMath.ONE_OVER_PI = 1.0 / Math.PI;

    /**
     * pi/2
     *
     * @type {Number}
     * @constant
     */
    CesiumMath.PI_OVER_TWO = Math.PI / 2.0;

    /**
     * pi/3
     *
     * @type {Number}
     * @constant
     */
    CesiumMath.PI_OVER_THREE = Math.PI / 3.0;

    /**
     * pi/4
     *
     * @type {Number}
     * @constant
     */
    CesiumMath.PI_OVER_FOUR = Math.PI / 4.0;

    /**
     * pi/6
     *
     * @type {Number}
     * @constant
     */
    CesiumMath.PI_OVER_SIX = Math.PI / 6.0;

    /**
     * 3pi/2
     *
     * @type {Number}
     * @constant
     */
    CesiumMath.THREE_PI_OVER_TWO = 3.0 * Math.PI / 2.0;

    /**
     * 2pi
     *
     * @type {Number}
     * @constant
     */
    CesiumMath.TWO_PI = 2.0 * Math.PI;

    /**
     * 1/2pi
     *
     * @type {Number}
     * @constant
     */
    CesiumMath.ONE_OVER_TWO_PI = 1.0 / (2.0 * Math.PI);

    /**
     * The number of radians in a degree.
     *
     * @type {Number}
     * @constant
     * @default Math.PI / 180.0
     */
    CesiumMath.RADIANS_PER_DEGREE = Math.PI / 180.0;

    /**
     * The number of degrees in a radian.
     *
     * @type {Number}
     * @constant
     * @default 180.0 / Math.PI
     */
    CesiumMath.DEGREES_PER_RADIAN = 180.0 / Math.PI;

    /**
     * The number of radians in an arc second.
     *
     * @type {Number}
     * @constant
     * @default {@link CesiumMath.RADIANS_PER_DEGREE} / 3600.0
     */
    CesiumMath.RADIANS_PER_ARCSECOND = CesiumMath.RADIANS_PER_DEGREE / 3600.0;

    /**
     * Converts degrees to radians.
     * @param {Number} degrees The angle to convert in degrees.
     * @returns {Number} The corresponding angle in radians.
     */
    CesiumMath.toRadians = function(degrees) {
                if (!defined(degrees)) {
            throw new DeveloperError('degrees is required.');
        }
                return degrees * CesiumMath.RADIANS_PER_DEGREE;
    };

    /**
     * Converts radians to degrees.
     * @param {Number} radians The angle to convert in radians.
     * @returns {Number} The corresponding angle in degrees.
     */
    CesiumMath.toDegrees = function(radians) {
                if (!defined(radians)) {
            throw new DeveloperError('radians is required.');
        }
                return radians * CesiumMath.DEGREES_PER_RADIAN;
    };

    /**
     * Converts a longitude value, in radians, to the range [<code>-Math.PI</code>, <code>Math.PI</code>).
     *
     * @param {Number} angle The longitude value, in radians, to convert to the range [<code>-Math.PI</code>, <code>Math.PI</code>).
     * @returns {Number} The equivalent longitude value in the range [<code>-Math.PI</code>, <code>Math.PI</code>).
     *
     * @example
     * // Convert 270 degrees to -90 degrees longitude
     * var longitude = Cesium.Math.convertLongitudeRange(Cesium.Math.toRadians(270.0));
     */
    CesiumMath.convertLongitudeRange = function(angle) {
                if (!defined(angle)) {
            throw new DeveloperError('angle is required.');
        }
                var twoPi = CesiumMath.TWO_PI;

        var simplified = angle - Math.floor(angle / twoPi) * twoPi;

        if (simplified < -Math.PI) {
            return simplified + twoPi;
        }
        if (simplified >= Math.PI) {
            return simplified - twoPi;
        }

        return simplified;
    };

    /**
     * Convenience function that clamps a latitude value, in radians, to the range [<code>-Math.PI/2</code>, <code>Math.PI/2</code>).
     * Useful for sanitizing data before use in objects requiring correct range.
     *
     * @param {Number} angle The latitude value, in radians, to clamp to the range [<code>-Math.PI/2</code>, <code>Math.PI/2</code>).
     * @returns {Number} The latitude value clamped to the range [<code>-Math.PI/2</code>, <code>Math.PI/2</code>).
     *
     * @example
     * // Clamp 108 degrees latitude to 90 degrees latitude
     * var latitude = Cesium.Math.clampToLatitudeRange(Cesium.Math.toRadians(108.0));
     */
    CesiumMath.clampToLatitudeRange = function(angle) {
                if (!defined(angle)) {
            throw new DeveloperError('angle is required.');
        }
        
        return CesiumMath.clamp(angle, -1*CesiumMath.PI_OVER_TWO, CesiumMath.PI_OVER_TWO);
    };

    /**
     * Produces an angle in the range -Pi <= angle <= Pi which is equivalent to the provided angle.
     *
     * @param {Number} angle in radians
     * @returns {Number} The angle in the range [<code>-CesiumMath.PI</code>, <code>CesiumMath.PI</code>].
     */
    CesiumMath.negativePiToPi = function(angle) {
                if (!defined(angle)) {
            throw new DeveloperError('angle is required.');
        }
                return CesiumMath.zeroToTwoPi(angle + CesiumMath.PI) - CesiumMath.PI;
    };

    /**
     * Produces an angle in the range 0 <= angle <= 2Pi which is equivalent to the provided angle.
     *
     * @param {Number} angle in radians
     * @returns {Number} The angle in the range [0, <code>CesiumMath.TWO_PI</code>].
     */
    CesiumMath.zeroToTwoPi = function(angle) {
                if (!defined(angle)) {
            throw new DeveloperError('angle is required.');
        }
                var mod = CesiumMath.mod(angle, CesiumMath.TWO_PI);
        if (Math.abs(mod) < CesiumMath.EPSILON14 && Math.abs(angle) > CesiumMath.EPSILON14) {
            return CesiumMath.TWO_PI;
        }
        return mod;
    };

    /**
     * The modulo operation that also works for negative dividends.
     *
     * @param {Number} m The dividend.
     * @param {Number} n The divisor.
     * @returns {Number} The remainder.
     */
    CesiumMath.mod = function(m, n) {
                if (!defined(m)) {
            throw new DeveloperError('m is required.');
        }
        if (!defined(n)) {
            throw new DeveloperError('n is required.');
        }
                return ((m % n) + n) % n;
    };

    /**
     * Determines if two values are equal using an absolute or relative tolerance test. This is useful
     * to avoid problems due to roundoff error when comparing floating-point values directly. The values are
     * first compared using an absolute tolerance test. If that fails, a relative tolerance test is performed.
     * Use this test if you are unsure of the magnitudes of left and right.
     *
     * @param {Number} left The first value to compare.
     * @param {Number} right The other value to compare.
     * @param {Number} relativeEpsilon The maximum inclusive delta between <code>left</code> and <code>right</code> for the relative tolerance test.
     * @param {Number} [absoluteEpsilon=relativeEpsilon] The maximum inclusive delta between <code>left</code> and <code>right</code> for the absolute tolerance test.
     * @returns {Boolean} <code>true</code> if the values are equal within the epsilon; otherwise, <code>false</code>.
     *
     * @example
     * var a = Cesium.Math.equalsEpsilon(0.0, 0.01, Cesium.Math.EPSILON2); // true
     * var b = Cesium.Math.equalsEpsilon(0.0, 0.1, Cesium.Math.EPSILON2);  // false
     * var c = Cesium.Math.equalsEpsilon(3699175.1634344, 3699175.2, Cesium.Math.EPSILON7); // true
     * var d = Cesium.Math.equalsEpsilon(3699175.1634344, 3699175.2, Cesium.Math.EPSILON9); // false
     */
    CesiumMath.equalsEpsilon = function(left, right, relativeEpsilon, absoluteEpsilon) {
                if (!defined(left)) {
            throw new DeveloperError('left is required.');
        }
        if (!defined(right)) {
            throw new DeveloperError('right is required.');
        }
        if (!defined(relativeEpsilon)) {
            throw new DeveloperError('relativeEpsilon is required.');
        }
                absoluteEpsilon = defaultValue(absoluteEpsilon, relativeEpsilon);
        var absDiff = Math.abs(left - right);
        return absDiff <= absoluteEpsilon || absDiff <= relativeEpsilon * Math.max(Math.abs(left), Math.abs(right));
    };

    /**
     * Determines if the left value is less than the right value. If the two values are within
     * <code>absoluteEpsilon</code> of each other, they are considered equal and this function returns false.
     *
     * @param {Number} left The first number to compare.
     * @param {Number} right The second number to compare.
     * @param {Number} absoluteEpsilon The absolute epsilon to use in comparison.
     * @returns {Boolean} <code>true</code> if <code>left</code> is less than <code>right</code> by more than
     *          <code>absoluteEpsilon<code>. <code>false</code> if <code>left</code> is greater or if the two
     *          values are nearly equal.
     */
    CesiumMath.lessThan = function(left, right, absoluteEpsilon) {
                if (!defined(left)) {
            throw new DeveloperError('first is required.');
        }
        if (!defined(right)) {
            throw new DeveloperError('second is required.');
        }
        if (!defined(absoluteEpsilon)) {
            throw new DeveloperError('relativeEpsilon is required.');
        }
                return left - right < -absoluteEpsilon;
    };

    /**
     * Determines if the left value is less than or equal to the right value. If the two values are within
     * <code>absoluteEpsilon</code> of each other, they are considered equal and this function returns true.
     *
     * @param {Number} left The first number to compare.
     * @param {Number} right The second number to compare.
     * @param {Number} absoluteEpsilon The absolute epsilon to use in comparison.
     * @returns {Boolean} <code>true</code> if <code>left</code> is less than <code>right</code> or if the
     *          the values are nearly equal.
     */
    CesiumMath.lessThanOrEquals = function(left, right, absoluteEpsilon) {
                if (!defined(left)) {
            throw new DeveloperError('first is required.');
        }
        if (!defined(right)) {
            throw new DeveloperError('second is required.');
        }
        if (!defined(absoluteEpsilon)) {
            throw new DeveloperError('relativeEpsilon is required.');
        }
                return left - right < absoluteEpsilon;
    };

    /**
     * Determines if the left value is greater the right value. If the two values are within
     * <code>absoluteEpsilon</code> of each other, they are considered equal and this function returns false.
     *
     * @param {Number} left The first number to compare.
     * @param {Number} right The second number to compare.
     * @param {Number} absoluteEpsilon The absolute epsilon to use in comparison.
     * @returns {Boolean} <code>true</code> if <code>left</code> is greater than <code>right</code> by more than
     *          <code>absoluteEpsilon<code>. <code>false</code> if <code>left</code> is less or if the two
     *          values are nearly equal.
     */
    CesiumMath.greaterThan = function(left, right, absoluteEpsilon) {
                if (!defined(left)) {
            throw new DeveloperError('first is required.');
        }
        if (!defined(right)) {
            throw new DeveloperError('second is required.');
        }
        if (!defined(absoluteEpsilon)) {
            throw new DeveloperError('relativeEpsilon is required.');
        }
                return left - right > absoluteEpsilon;
    };

    /**
     * Determines if the left value is greater than or equal to the right value. If the two values are within
     * <code>absoluteEpsilon</code> of each other, they are considered equal and this function returns true.
     *
     * @param {Number} left The first number to compare.
     * @param {Number} right The second number to compare.
     * @param {Number} absoluteEpsilon The absolute epsilon to use in comparison.
     * @returns {Boolean} <code>true</code> if <code>left</code> is greater than <code>right</code> or if the
     *          the values are nearly equal.
     */
    CesiumMath.greaterThanOrEquals = function(left, right, absoluteEpsilon) {
                if (!defined(left)) {
            throw new DeveloperError('first is required.');
        }
        if (!defined(right)) {
            throw new DeveloperError('second is required.');
        }
        if (!defined(absoluteEpsilon)) {
            throw new DeveloperError('relativeEpsilon is required.');
        }
                return left - right > -absoluteEpsilon;
    };

    var factorials = [1];

    /**
     * Computes the factorial of the provided number.
     *
     * @param {Number} n The number whose factorial is to be computed.
     * @returns {Number} The factorial of the provided number or undefined if the number is less than 0.
     *
     * @exception {DeveloperError} A number greater than or equal to 0 is required.
     *
     *
     * @example
     * //Compute 7!, which is equal to 5040
     * var computedFactorial = Cesium.Math.factorial(7);
     *
     * @see {@link http://en.wikipedia.org/wiki/Factorial|Factorial on Wikipedia}
     */
    CesiumMath.factorial = function(n) {
                if (typeof n !== 'number' || n < 0) {
            throw new DeveloperError('A number greater than or equal to 0 is required.');
        }
        
        var length = factorials.length;
        if (n >= length) {
            var sum = factorials[length - 1];
            for (var i = length; i <= n; i++) {
                var next = sum * i;
                factorials.push(next);
                sum = next;
            }
        }
        return factorials[n];
    };

    /**
     * Increments a number with a wrapping to a minimum value if the number exceeds the maximum value.
     *
     * @param {Number} [n] The number to be incremented.
     * @param {Number} [maximumValue] The maximum incremented value before rolling over to the minimum value.
     * @param {Number} [minimumValue=0.0] The number reset to after the maximum value has been exceeded.
     * @returns {Number} The incremented number.
     *
     * @exception {DeveloperError} Maximum value must be greater than minimum value.
     *
     * @example
     * var n = Cesium.Math.incrementWrap(5, 10, 0); // returns 6
     * var n = Cesium.Math.incrementWrap(10, 10, 0); // returns 0
     */
    CesiumMath.incrementWrap = function(n, maximumValue, minimumValue) {
        minimumValue = defaultValue(minimumValue, 0.0);

                if (!defined(n)) {
            throw new DeveloperError('n is required.');
        }
        if (maximumValue <= minimumValue) {
            throw new DeveloperError('maximumValue must be greater than minimumValue.');
        }
        
        ++n;
        if (n > maximumValue) {
            n = minimumValue;
        }
        return n;
    };

    /**
     * Determines if a positive integer is a power of two.
     *
     * @param {Number} n The positive integer to test.
     * @returns {Boolean} <code>true</code> if the number if a power of two; otherwise, <code>false</code>.
     *
     * @exception {DeveloperError} A number greater than or equal to 0 is required.
     *
     * @example
     * var t = Cesium.Math.isPowerOfTwo(16); // true
     * var f = Cesium.Math.isPowerOfTwo(20); // false
     */
    CesiumMath.isPowerOfTwo = function(n) {
                if (typeof n !== 'number' || n < 0) {
            throw new DeveloperError('A number greater than or equal to 0 is required.');
        }
        
        return (n !== 0) && ((n & (n - 1)) === 0);
    };

    /**
     * Computes the next power-of-two integer greater than or equal to the provided positive integer.
     *
     * @param {Number} n The positive integer to test.
     * @returns {Number} The next power-of-two integer.
     *
     * @exception {DeveloperError} A number greater than or equal to 0 is required.
     *
     * @example
     * var n = Cesium.Math.nextPowerOfTwo(29); // 32
     * var m = Cesium.Math.nextPowerOfTwo(32); // 32
     */
    CesiumMath.nextPowerOfTwo = function(n) {
                if (typeof n !== 'number' || n < 0) {
            throw new DeveloperError('A number greater than or equal to 0 is required.');
        }
        
        // From http://graphics.stanford.edu/~seander/bithacks.html#RoundUpPowerOf2
        --n;
        n |= n >> 1;
        n |= n >> 2;
        n |= n >> 4;
        n |= n >> 8;
        n |= n >> 16;
        ++n;

        return n;
    };

    /**
     * Constraint a value to lie between two values.
     *
     * @param {Number} value The value to constrain.
     * @param {Number} min The minimum value.
     * @param {Number} max The maximum value.
     * @returns {Number} The value clamped so that min <= value <= max.
     */
    CesiumMath.clamp = function(value, min, max) {
                if (!defined(value)) {
            throw new DeveloperError('value is required');
        }
        if (!defined(min)) {
            throw new DeveloperError('min is required.');
        }
        if (!defined(max)) {
            throw new DeveloperError('max is required.');
        }
                return value < min ? min : value > max ? max : value;
    };

    var randomNumberGenerator = new MersenneTwister();

    /**
     * Sets the seed used by the random number generator
     * in {@link CesiumMath#nextRandomNumber}.
     *
     * @param {Number} seed An integer used as the seed.
     */
    CesiumMath.setRandomNumberSeed = function(seed) {
                if (!defined(seed)) {
            throw new DeveloperError('seed is required.');
        }
        
        randomNumberGenerator = new MersenneTwister(seed);
    };

    /**
     * Generates a random floating point number in the range of [0.0, 1.0)
     * using a Mersenne twister.
     *
     * @returns {Number} A random number in the range of [0.0, 1.0).
     *
     * @see CesiumMath.setRandomNumberSeed
     * @see {@link http://en.wikipedia.org/wiki/Mersenne_twister|Mersenne twister on Wikipedia}
     */
    CesiumMath.nextRandomNumber = function() {
        return randomNumberGenerator.random();
    };

    /**
     * Generates a random number between two numbers.
     *
     * @param {Number} min The minimum value.
     * @param {Number} max The maximum value.
     * @returns {Number} A random number between the min and max.
     */
    CesiumMath.randomBetween = function(min, max) {
        return CesiumMath.nextRandomNumber() * (max - min) + min;
    };

    /**
     * Computes <code>Math.acos(value)</code>, but first clamps <code>value</code> to the range [-1.0, 1.0]
     * so that the function will never return NaN.
     *
     * @param {Number} value The value for which to compute acos.
     * @returns {Number} The acos of the value if the value is in the range [-1.0, 1.0], or the acos of -1.0 or 1.0,
     *          whichever is closer, if the value is outside the range.
     */
    CesiumMath.acosClamped = function(value) {
                if (!defined(value)) {
            throw new DeveloperError('value is required.');
        }
                return Math.acos(CesiumMath.clamp(value, -1.0, 1.0));
    };

    /**
     * Computes <code>Math.asin(value)</code>, but first clamps <code>value</code> to the range [-1.0, 1.0]
     * so that the function will never return NaN.
     *
     * @param {Number} value The value for which to compute asin.
     * @returns {Number} The asin of the value if the value is in the range [-1.0, 1.0], or the asin of -1.0 or 1.0,
     *          whichever is closer, if the value is outside the range.
     */
    CesiumMath.asinClamped = function(value) {
                if (!defined(value)) {
            throw new DeveloperError('value is required.');
        }
                return Math.asin(CesiumMath.clamp(value, -1.0, 1.0));
    };

    /**
     * Finds the chord length between two points given the circle's radius and the angle between the points.
     *
     * @param {Number} angle The angle between the two points.
     * @param {Number} radius The radius of the circle.
     * @returns {Number} The chord length.
     */
    CesiumMath.chordLength = function(angle, radius) {
                if (!defined(angle)) {
            throw new DeveloperError('angle is required.');
        }
        if (!defined(radius)) {
            throw new DeveloperError('radius is required.');
        }
                return 2.0 * radius * Math.sin(angle * 0.5);
    };

    /**
     * Finds the logarithm of a number to a base.
     *
     * @param {Number} number The number.
     * @param {Number} base The base.
     * @returns {Number} The result.
     */
    CesiumMath.logBase = function(number, base) {
                if (!defined(number)) {
            throw new DeveloperError('number is required.');
        }
        if (!defined(base)) {
            throw new DeveloperError('base is required.');
        }
                return Math.log(number) / Math.log(base);
    };

    /**
     * Finds the cube root of a number.
     * Returns NaN if <code>number</code> is not provided.
     *
     * @function
     * @param {Number} [number] The number.
     * @returns {Number} The result.
     */
    CesiumMath.cbrt = defaultValue(Math.cbrt, function cbrt(number) {
        var result = Math.pow(Math.abs(number), 1.0 / 3.0);
        return number < 0.0 ? -result : result;
    });

    /**
     * Finds the base 2 logarithm of a number.
     *
     * @function
     * @param {Number} number The number.
     * @returns {Number} The result.
     */
    CesiumMath.log2 = defaultValue(Math.log2, function log2(number) {
        return Math.log(number) * Math.LOG2E;
    });

    /**
     * @private
     */
    CesiumMath.fog = function(distanceToCamera, density) {
        var scalar = distanceToCamera * density;
        return 1.0 - Math.exp(-(scalar * scalar));
    };

    /**
     * Computes a fast approximation of Atan for input in the range [-1, 1].
     *
     * Based on Michal Drobot's approximation from ShaderFastLibs,
     * which in turn is based on "Efficient approximations for the arctangent function,"
     * Rajan, S. Sichun Wang Inkol, R. Joyal, A., May 2006.
     * Adapted from ShaderFastLibs under MIT License.
     *
     * @param {Number} x An input number in the range [-1, 1]
     * @returns {Number} An approximation of atan(x)
     */
    CesiumMath.fastApproximateAtan = function(x) {
                Check.typeOf.number('x', x);
        
        return x * (-0.1784 * Math.abs(x) - 0.0663 * x * x + 1.0301);
    };

    /**
     * Computes a fast approximation of Atan2(x, y) for arbitrary input scalars.
     *
     * Range reduction math based on nvidia's cg reference implementation: http://developer.download.nvidia.com/cg/atan2.html
     *
     * @param {Number} x An input number that isn't zero if y is zero.
     * @param {Number} y An input number that isn't zero if x is zero.
     * @returns {Number} An approximation of atan2(x, y)
     */
    CesiumMath.fastApproximateAtan2 = function(x, y) {
                Check.typeOf.number('x', x);
        Check.typeOf.number('y', y);
        
        // atan approximations are usually only reliable over [-1, 1]
        // So reduce the range by flipping whether x or y is on top based on which is bigger.
        var opposite;
        var adjacent;
        var t = Math.abs(x); // t used as swap and atan result.
        opposite = Math.abs(y);
        adjacent = Math.max(t, opposite);
        opposite = Math.min(t, opposite);

        var oppositeOverAdjacent = opposite / adjacent;
                if (isNaN(oppositeOverAdjacent)) {
            throw new DeveloperError('either x or y must be nonzero');
        }
                t = CesiumMath.fastApproximateAtan(oppositeOverAdjacent);

        // Undo range reduction
        t = Math.abs(y) > Math.abs(x) ? CesiumMath.PI_OVER_TWO - t : t;
        t = x < 0.0 ?  CesiumMath.PI - t : t;
        t = y < 0.0 ? -t : t;
        return t;
    };

    return CesiumMath;
});

define('Core/Cartesian3',[
        './Check',
        './defaultValue',
        './defined',
        './DeveloperError',
        './freezeObject',
        './Math'
    ], function(
        Check,
        defaultValue,
        defined,
        DeveloperError,
        freezeObject,
        CesiumMath) {
    'use strict';

    /**
     * A 3D Cartesian point.
     * @alias Cartesian3
     * @constructor
     *
     * @param {Number} [x=0.0] The X component.
     * @param {Number} [y=0.0] The Y component.
     * @param {Number} [z=0.0] The Z component.
     *
     * @see Cartesian2
     * @see Cartesian4
     * @see Packable
     */
    function Cartesian3(x, y, z) {
        /**
         * The X component.
         * @type {Number}
         * @default 0.0
         */
        this.x = defaultValue(x, 0.0);

        /**
         * The Y component.
         * @type {Number}
         * @default 0.0
         */
        this.y = defaultValue(y, 0.0);

        /**
         * The Z component.
         * @type {Number}
         * @default 0.0
         */
        this.z = defaultValue(z, 0.0);
    }

    /**
     * Converts the provided Spherical into Cartesian3 coordinates.
     *
     * @param {Spherical} spherical The Spherical to be converted to Cartesian3.
     * @param {Cartesian3} [result] The object onto which to store the result.
     * @returns {Cartesian3} The modified result parameter or a new Cartesian3 instance if one was not provided.
     */
    Cartesian3.fromSpherical = function(spherical, result) {
                Check.typeOf.object('spherical', spherical);
        
        if (!defined(result)) {
            result = new Cartesian3();
        }

        var clock = spherical.clock;
        var cone = spherical.cone;
        var magnitude = defaultValue(spherical.magnitude, 1.0);
        var radial = magnitude * Math.sin(cone);
        result.x = radial * Math.cos(clock);
        result.y = radial * Math.sin(clock);
        result.z = magnitude * Math.cos(cone);
        return result;
    };

    /**
     * Creates a Cartesian3 instance from x, y and z coordinates.
     *
     * @param {Number} x The x coordinate.
     * @param {Number} y The y coordinate.
     * @param {Number} z The z coordinate.
     * @param {Cartesian3} [result] The object onto which to store the result.
     * @returns {Cartesian3} The modified result parameter or a new Cartesian3 instance if one was not provided.
     */
    Cartesian3.fromElements = function(x, y, z, result) {
        if (!defined(result)) {
            return new Cartesian3(x, y, z);
        }

        result.x = x;
        result.y = y;
        result.z = z;
        return result;
    };

    /**
     * Duplicates a Cartesian3 instance.
     *
     * @param {Cartesian3} cartesian The Cartesian to duplicate.
     * @param {Cartesian3} [result] The object onto which to store the result.
     * @returns {Cartesian3} The modified result parameter or a new Cartesian3 instance if one was not provided. (Returns undefined if cartesian is undefined)
     */
    Cartesian3.clone = function(cartesian, result) {
        if (!defined(cartesian)) {
            return undefined;
        }
        if (!defined(result)) {
            return new Cartesian3(cartesian.x, cartesian.y, cartesian.z);
        }

        result.x = cartesian.x;
        result.y = cartesian.y;
        result.z = cartesian.z;
        return result;
    };

    /**
     * Creates a Cartesian3 instance from an existing Cartesian4.  This simply takes the
     * x, y, and z properties of the Cartesian4 and drops w.
     * @function
     *
     * @param {Cartesian4} cartesian The Cartesian4 instance to create a Cartesian3 instance from.
     * @param {Cartesian3} [result] The object onto which to store the result.
     * @returns {Cartesian3} The modified result parameter or a new Cartesian3 instance if one was not provided.
     */
    Cartesian3.fromCartesian4 = Cartesian3.clone;

    /**
     * The number of elements used to pack the object into an array.
     * @type {Number}
     */
    Cartesian3.packedLength = 3;

    /**
     * Stores the provided instance into the provided array.
     *
     * @param {Cartesian3} value The value to pack.
     * @param {Number[]} array The array to pack into.
     * @param {Number} [startingIndex=0] The index into the array at which to start packing the elements.
     *
     * @returns {Number[]} The array that was packed into
     */
    Cartesian3.pack = function(value, array, startingIndex) {
                Check.typeOf.object('value', value);
        Check.defined('array', array);
        
        startingIndex = defaultValue(startingIndex, 0);

        array[startingIndex++] = value.x;
        array[startingIndex++] = value.y;
        array[startingIndex] = value.z;

        return array;
    };

    /**
     * Retrieves an instance from a packed array.
     *
     * @param {Number[]} array The packed array.
     * @param {Number} [startingIndex=0] The starting index of the element to be unpacked.
     * @param {Cartesian3} [result] The object into which to store the result.
     * @returns {Cartesian3} The modified result parameter or a new Cartesian3 instance if one was not provided.
     */
    Cartesian3.unpack = function(array, startingIndex, result) {
                Check.defined('array', array);
        
        startingIndex = defaultValue(startingIndex, 0);

        if (!defined(result)) {
            result = new Cartesian3();
        }
        result.x = array[startingIndex++];
        result.y = array[startingIndex++];
        result.z = array[startingIndex];
        return result;
    };

    /**
     * Flattens an array of Cartesian3s into an array of components.
     *
     * @param {Cartesian3[]} array The array of cartesians to pack.
     * @param {Number[]} [result] The array onto which to store the result.
     * @returns {Number[]} The packed array.
     */
    Cartesian3.packArray = function(array, result) {
                Check.defined('array', array);
        
        var length = array.length;
        if (!defined(result)) {
            result = new Array(length * 3);
        } else {
            result.length = length * 3;
        }

        for (var i = 0; i < length; ++i) {
            Cartesian3.pack(array[i], result, i * 3);
        }
        return result;
    };

    /**
     * Unpacks an array of cartesian components into an array of Cartesian3s.
     *
     * @param {Number[]} array The array of components to unpack.
     * @param {Cartesian3[]} [result] The array onto which to store the result.
     * @returns {Cartesian3[]} The unpacked array.
     */
    Cartesian3.unpackArray = function(array, result) {
                Check.defined('array', array);
        Check.typeOf.number.greaterThanOrEquals('array.length', array.length, 3);
        if (array.length % 3 !== 0) {
            throw new DeveloperError('array length must be a multiple of 3.');
        }
        
        var length = array.length;
        if (!defined(result)) {
            result = new Array(length / 3);
        } else {
            result.length = length / 3;
        }

        for (var i = 0; i < length; i += 3) {
            var index = i / 3;
            result[index] = Cartesian3.unpack(array, i, result[index]);
        }
        return result;
    };

    /**
     * Creates a Cartesian3 from three consecutive elements in an array.
     * @function
     *
     * @param {Number[]} array The array whose three consecutive elements correspond to the x, y, and z components, respectively.
     * @param {Number} [startingIndex=0] The offset into the array of the first element, which corresponds to the x component.
     * @param {Cartesian3} [result] The object onto which to store the result.
     * @returns {Cartesian3} The modified result parameter or a new Cartesian3 instance if one was not provided.
     *
     * @example
     * // Create a Cartesian3 with (1.0, 2.0, 3.0)
     * var v = [1.0, 2.0, 3.0];
     * var p = Cesium.Cartesian3.fromArray(v);
     *
     * // Create a Cartesian3 with (1.0, 2.0, 3.0) using an offset into an array
     * var v2 = [0.0, 0.0, 1.0, 2.0, 3.0];
     * var p2 = Cesium.Cartesian3.fromArray(v2, 2);
     */
    Cartesian3.fromArray = Cartesian3.unpack;

    /**
     * Computes the value of the maximum component for the supplied Cartesian.
     *
     * @param {Cartesian3} cartesian The cartesian to use.
     * @returns {Number} The value of the maximum component.
     */
    Cartesian3.maximumComponent = function(cartesian) {
                Check.typeOf.object('cartesian', cartesian);
        
        return Math.max(cartesian.x, cartesian.y, cartesian.z);
    };

    /**
     * Computes the value of the minimum component for the supplied Cartesian.
     *
     * @param {Cartesian3} cartesian The cartesian to use.
     * @returns {Number} The value of the minimum component.
     */
    Cartesian3.minimumComponent = function(cartesian) {
                Check.typeOf.object('cartesian', cartesian);
        
        return Math.min(cartesian.x, cartesian.y, cartesian.z);
    };

    /**
     * Compares two Cartesians and computes a Cartesian which contains the minimum components of the supplied Cartesians.
     *
     * @param {Cartesian3} first A cartesian to compare.
     * @param {Cartesian3} second A cartesian to compare.
     * @param {Cartesian3} result The object into which to store the result.
     * @returns {Cartesian3} A cartesian with the minimum components.
     */
    Cartesian3.minimumByComponent = function(first, second, result) {
                Check.typeOf.object('first', first);
        Check.typeOf.object('second', second);
        Check.typeOf.object('result', result);
        
        result.x = Math.min(first.x, second.x);
        result.y = Math.min(first.y, second.y);
        result.z = Math.min(first.z, second.z);

        return result;
    };

    /**
     * Compares two Cartesians and computes a Cartesian which contains the maximum components of the supplied Cartesians.
     *
     * @param {Cartesian3} first A cartesian to compare.
     * @param {Cartesian3} second A cartesian to compare.
     * @param {Cartesian3} result The object into which to store the result.
     * @returns {Cartesian3} A cartesian with the maximum components.
     */
    Cartesian3.maximumByComponent = function(first, second, result) {
                Check.typeOf.object('first', first);
        Check.typeOf.object('second', second);
        Check.typeOf.object('result', result);
        
        result.x = Math.max(first.x, second.x);
        result.y = Math.max(first.y, second.y);
        result.z = Math.max(first.z, second.z);
        return result;
    };

    /**
     * Computes the provided Cartesian's squared magnitude.
     *
     * @param {Cartesian3} cartesian The Cartesian instance whose squared magnitude is to be computed.
     * @returns {Number} The squared magnitude.
     */
    Cartesian3.magnitudeSquared = function(cartesian) {
                Check.typeOf.object('cartesian', cartesian);
        
        return cartesian.x * cartesian.x + cartesian.y * cartesian.y + cartesian.z * cartesian.z;
    };

    /**
     * Computes the Cartesian's magnitude (length).
     *
     * @param {Cartesian3} cartesian The Cartesian instance whose magnitude is to be computed.
     * @returns {Number} The magnitude.
     */
    Cartesian3.magnitude = function(cartesian) {
        return Math.sqrt(Cartesian3.magnitudeSquared(cartesian));
    };

    var distanceScratch = new Cartesian3();

    /**
     * Computes the distance between two points.
     *
     * @param {Cartesian3} left The first point to compute the distance from.
     * @param {Cartesian3} right The second point to compute the distance to.
     * @returns {Number} The distance between two points.
     *
     * @example
     * // Returns 1.0
     * var d = Cesium.Cartesian3.distance(new Cesium.Cartesian3(1.0, 0.0, 0.0), new Cesium.Cartesian3(2.0, 0.0, 0.0));
     */
    Cartesian3.distance = function(left, right) {
                Check.typeOf.object('left', left);
        Check.typeOf.object('right', right);
        
        Cartesian3.subtract(left, right, distanceScratch);
        return Cartesian3.magnitude(distanceScratch);
    };

    /**
     * Computes the squared distance between two points.  Comparing squared distances
     * using this function is more efficient than comparing distances using {@link Cartesian3#distance}.
     *
     * @param {Cartesian3} left The first point to compute the distance from.
     * @param {Cartesian3} right The second point to compute the distance to.
     * @returns {Number} The distance between two points.
     *
     * @example
     * // Returns 4.0, not 2.0
     * var d = Cesium.Cartesian3.distanceSquared(new Cesium.Cartesian3(1.0, 0.0, 0.0), new Cesium.Cartesian3(3.0, 0.0, 0.0));
     */
    Cartesian3.distanceSquared = function(left, right) {
                Check.typeOf.object('left', left);
        Check.typeOf.object('right', right);
        
        Cartesian3.subtract(left, right, distanceScratch);
        return Cartesian3.magnitudeSquared(distanceScratch);
    };

    /**
     * Computes the normalized form of the supplied Cartesian.
     *
     * @param {Cartesian3} cartesian The Cartesian to be normalized.
     * @param {Cartesian3} result The object onto which to store the result.
     * @returns {Cartesian3} The modified result parameter.
     */
    Cartesian3.normalize = function(cartesian, result) {
                Check.typeOf.object('cartesian', cartesian);
        Check.typeOf.object('result', result);
        
        var magnitude = Cartesian3.magnitude(cartesian);

        result.x = cartesian.x / magnitude;
        result.y = cartesian.y / magnitude;
        result.z = cartesian.z / magnitude;

                if (isNaN(result.x) || isNaN(result.y) || isNaN(result.z)) {
            throw new DeveloperError('normalized result is not a number');
        }
        
        return result;
    };

    /**
     * Computes the dot (scalar) product of two Cartesians.
     *
     * @param {Cartesian3} left The first Cartesian.
     * @param {Cartesian3} right The second Cartesian.
     * @returns {Number} The dot product.
     */
    Cartesian3.dot = function(left, right) {
                Check.typeOf.object('left', left);
        Check.typeOf.object('right', right);
        
        return left.x * right.x + left.y * right.y + left.z * right.z;
    };

    /**
     * Computes the componentwise product of two Cartesians.
     *
     * @param {Cartesian3} left The first Cartesian.
     * @param {Cartesian3} right The second Cartesian.
     * @param {Cartesian3} result The object onto which to store the result.
     * @returns {Cartesian3} The modified result parameter.
     */
    Cartesian3.multiplyComponents = function(left, right, result) {
                Check.typeOf.object('left', left);
        Check.typeOf.object('right', right);
        Check.typeOf.object('result', result);
        
        result.x = left.x * right.x;
        result.y = left.y * right.y;
        result.z = left.z * right.z;
        return result;
    };

    /**
     * Computes the componentwise quotient of two Cartesians.
     *
     * @param {Cartesian3} left The first Cartesian.
     * @param {Cartesian3} right The second Cartesian.
     * @param {Cartesian3} result The object onto which to store the result.
     * @returns {Cartesian3} The modified result parameter.
     */
    Cartesian3.divideComponents = function(left, right, result) {
                Check.typeOf.object('left', left);
        Check.typeOf.object('right', right);
        Check.typeOf.object('result', result);
        
        result.x = left.x / right.x;
        result.y = left.y / right.y;
        result.z = left.z / right.z;
        return result;
    };

    /**
     * Computes the componentwise sum of two Cartesians.
     *
     * @param {Cartesian3} left The first Cartesian.
     * @param {Cartesian3} right The second Cartesian.
     * @param {Cartesian3} result The object onto which to store the result.
     * @returns {Cartesian3} The modified result parameter.
     */
    Cartesian3.add = function(left, right, result) {
                Check.typeOf.object('left', left);
        Check.typeOf.object('right', right);
        Check.typeOf.object('result', result);
        
        result.x = left.x + right.x;
        result.y = left.y + right.y;
        result.z = left.z + right.z;
        return result;
    };

    /**
     * Computes the componentwise difference of two Cartesians.
     *
     * @param {Cartesian3} left The first Cartesian.
     * @param {Cartesian3} right The second Cartesian.
     * @param {Cartesian3} result The object onto which to store the result.
     * @returns {Cartesian3} The modified result parameter.
     */
    Cartesian3.subtract = function(left, right, result) {
                Check.typeOf.object('left', left);
        Check.typeOf.object('right', right);
        Check.typeOf.object('result', result);
        
        result.x = left.x - right.x;
        result.y = left.y - right.y;
        result.z = left.z - right.z;
        return result;
    };

    /**
     * Multiplies the provided Cartesian componentwise by the provided scalar.
     *
     * @param {Cartesian3} cartesian The Cartesian to be scaled.
     * @param {Number} scalar The scalar to multiply with.
     * @param {Cartesian3} result The object onto which to store the result.
     * @returns {Cartesian3} The modified result parameter.
     */
    Cartesian3.multiplyByScalar = function(cartesian, scalar, result) {
                Check.typeOf.object('cartesian', cartesian);
        Check.typeOf.number('scalar', scalar);
        Check.typeOf.object('result', result);
        
        result.x = cartesian.x * scalar;
        result.y = cartesian.y * scalar;
        result.z = cartesian.z * scalar;
        return result;
    };

    /**
     * Divides the provided Cartesian componentwise by the provided scalar.
     *
     * @param {Cartesian3} cartesian The Cartesian to be divided.
     * @param {Number} scalar The scalar to divide by.
     * @param {Cartesian3} result The object onto which to store the result.
     * @returns {Cartesian3} The modified result parameter.
     */
    Cartesian3.divideByScalar = function(cartesian, scalar, result) {
                Check.typeOf.object('cartesian', cartesian);
        Check.typeOf.number('scalar', scalar);
        Check.typeOf.object('result', result);
        
        result.x = cartesian.x / scalar;
        result.y = cartesian.y / scalar;
        result.z = cartesian.z / scalar;
        return result;
    };

    /**
     * Negates the provided Cartesian.
     *
     * @param {Cartesian3} cartesian The Cartesian to be negated.
     * @param {Cartesian3} result The object onto which to store the result.
     * @returns {Cartesian3} The modified result parameter.
     */
    Cartesian3.negate = function(cartesian, result) {
                Check.typeOf.object('cartesian', cartesian);
        Check.typeOf.object('result', result);
        
        result.x = -cartesian.x;
        result.y = -cartesian.y;
        result.z = -cartesian.z;
        return result;
    };

    /**
     * Computes the absolute value of the provided Cartesian.
     *
     * @param {Cartesian3} cartesian The Cartesian whose absolute value is to be computed.
     * @param {Cartesian3} result The object onto which to store the result.
     * @returns {Cartesian3} The modified result parameter.
     */
    Cartesian3.abs = function(cartesian, result) {
                Check.typeOf.object('cartesian', cartesian);
        Check.typeOf.object('result', result);
        
        result.x = Math.abs(cartesian.x);
        result.y = Math.abs(cartesian.y);
        result.z = Math.abs(cartesian.z);
        return result;
    };

    var lerpScratch = new Cartesian3();
    /**
     * Computes the linear interpolation or extrapolation at t using the provided cartesians.
     *
     * @param {Cartesian3} start The value corresponding to t at 0.0.
     * @param {Cartesian3} end The value corresponding to t at 1.0.
     * @param {Number} t The point along t at which to interpolate.
     * @param {Cartesian3} result The object onto which to store the result.
     * @returns {Cartesian3} The modified result parameter.
     */
    Cartesian3.lerp = function(start, end, t, result) {
                Check.typeOf.object('start', start);
        Check.typeOf.object('end', end);
        Check.typeOf.number('t', t);
        Check.typeOf.object('result', result);
        
        Cartesian3.multiplyByScalar(end, t, lerpScratch);
        result = Cartesian3.multiplyByScalar(start, 1.0 - t, result);
        return Cartesian3.add(lerpScratch, result, result);
    };

    var angleBetweenScratch = new Cartesian3();
    var angleBetweenScratch2 = new Cartesian3();
    /**
     * Returns the angle, in radians, between the provided Cartesians.
     *
     * @param {Cartesian3} left The first Cartesian.
     * @param {Cartesian3} right The second Cartesian.
     * @returns {Number} The angle between the Cartesians.
     */
    Cartesian3.angleBetween = function(left, right) {
                Check.typeOf.object('left', left);
        Check.typeOf.object('right', right);
        
        Cartesian3.normalize(left, angleBetweenScratch);
        Cartesian3.normalize(right, angleBetweenScratch2);
        var cosine = Cartesian3.dot(angleBetweenScratch, angleBetweenScratch2);
        var sine = Cartesian3.magnitude(Cartesian3.cross(angleBetweenScratch, angleBetweenScratch2, angleBetweenScratch));
        return Math.atan2(sine, cosine);
    };

    var mostOrthogonalAxisScratch = new Cartesian3();
    /**
     * Returns the axis that is most orthogonal to the provided Cartesian.
     *
     * @param {Cartesian3} cartesian The Cartesian on which to find the most orthogonal axis.
     * @param {Cartesian3} result The object onto which to store the result.
     * @returns {Cartesian3} The most orthogonal axis.
     */
    Cartesian3.mostOrthogonalAxis = function(cartesian, result) {
                Check.typeOf.object('cartesian', cartesian);
        Check.typeOf.object('result', result);
        
        var f = Cartesian3.normalize(cartesian, mostOrthogonalAxisScratch);
        Cartesian3.abs(f, f);

        if (f.x <= f.y) {
            if (f.x <= f.z) {
                result = Cartesian3.clone(Cartesian3.UNIT_X, result);
            } else {
                result = Cartesian3.clone(Cartesian3.UNIT_Z, result);
            }
        } else if (f.y <= f.z) {
            result = Cartesian3.clone(Cartesian3.UNIT_Y, result);
        } else {
            result = Cartesian3.clone(Cartesian3.UNIT_Z, result);
        }

        return result;
    };

    /**
     * Projects vector a onto vector b
     * @param {Cartesian3} a The vector that needs projecting
     * @param {Cartesian3} b The vector to project onto
     * @param {Cartesian3} result The result cartesian
     * @returns {Cartesian3} The modified result parameter
     */
    Cartesian3.projectVector = function(a, b, result) {
                Check.defined('a', a);
        Check.defined('b', b);
        Check.defined('result', result);
        
        var scalar = Cartesian3.dot(a, b) / Cartesian3.dot(b, b);
        return Cartesian3.multiplyByScalar(b, scalar, result);
    };

    /**
     * Compares the provided Cartesians componentwise and returns
     * <code>true</code> if they are equal, <code>false</code> otherwise.
     *
     * @param {Cartesian3} [left] The first Cartesian.
     * @param {Cartesian3} [right] The second Cartesian.
     * @returns {Boolean} <code>true</code> if left and right are equal, <code>false</code> otherwise.
     */
    Cartesian3.equals = function(left, right) {
            return (left === right) ||
              ((defined(left)) &&
               (defined(right)) &&
               (left.x === right.x) &&
               (left.y === right.y) &&
               (left.z === right.z));
    };

    /**
     * @private
     */
    Cartesian3.equalsArray = function(cartesian, array, offset) {
        return cartesian.x === array[offset] &&
               cartesian.y === array[offset + 1] &&
               cartesian.z === array[offset + 2];
    };

    /**
     * Compares the provided Cartesians componentwise and returns
     * <code>true</code> if they pass an absolute or relative tolerance test,
     * <code>false</code> otherwise.
     *
     * @param {Cartesian3} [left] The first Cartesian.
     * @param {Cartesian3} [right] The second Cartesian.
     * @param {Number} relativeEpsilon The relative epsilon tolerance to use for equality testing.
     * @param {Number} [absoluteEpsilon=relativeEpsilon] The absolute epsilon tolerance to use for equality testing.
     * @returns {Boolean} <code>true</code> if left and right are within the provided epsilon, <code>false</code> otherwise.
     */
    Cartesian3.equalsEpsilon = function(left, right, relativeEpsilon, absoluteEpsilon) {
        return (left === right) ||
               (defined(left) &&
                defined(right) &&
                CesiumMath.equalsEpsilon(left.x, right.x, relativeEpsilon, absoluteEpsilon) &&
                CesiumMath.equalsEpsilon(left.y, right.y, relativeEpsilon, absoluteEpsilon) &&
                CesiumMath.equalsEpsilon(left.z, right.z, relativeEpsilon, absoluteEpsilon));
    };

    /**
     * Computes the cross (outer) product of two Cartesians.
     *
     * @param {Cartesian3} left The first Cartesian.
     * @param {Cartesian3} right The second Cartesian.
     * @param {Cartesian3} result The object onto which to store the result.
     * @returns {Cartesian3} The cross product.
     */
    Cartesian3.cross = function(left, right, result) {
                Check.typeOf.object('left', left);
        Check.typeOf.object('right', right);
        Check.typeOf.object('result', result);
        
        var leftX = left.x;
        var leftY = left.y;
        var leftZ = left.z;
        var rightX = right.x;
        var rightY = right.y;
        var rightZ = right.z;

        var x = leftY * rightZ - leftZ * rightY;
        var y = leftZ * rightX - leftX * rightZ;
        var z = leftX * rightY - leftY * rightX;

        result.x = x;
        result.y = y;
        result.z = z;
        return result;
    };

    /**
     * Computes the midpoint between the right and left Cartesian.
     * @param {Cartesian3} left The first Cartesian.
     * @param {Cartesian3} right The second Cartesian.
     * @param {Cartesian3} result The object onto which to store the result.
     * @returns {Cartesian3} The midpoint.
     */
    Cartesian3.midpoint = function(left, right, result) {
                Check.typeOf.object('left', left);
        Check.typeOf.object('right', right);
        Check.typeOf.object('result', result);
        
        result.x = (left.x + right.x) * 0.5;
        result.y = (left.y + right.y) * 0.5;
        result.z = (left.z + right.z) * 0.5;

        return result;
    };

    /**
     * Returns a Cartesian3 position from longitude and latitude values given in degrees.
     *
     * @param {Number} longitude The longitude, in degrees
     * @param {Number} latitude The latitude, in degrees
     * @param {Number} [height=0.0] The height, in meters, above the ellipsoid.
     * @param {Ellipsoid} [ellipsoid=Ellipsoid.WGS84] The ellipsoid on which the position lies.
     * @param {Cartesian3} [result] The object onto which to store the result.
     * @returns {Cartesian3} The position
     *
     * @example
     * var position = Cesium.Cartesian3.fromDegrees(-115.0, 37.0);
     */
    Cartesian3.fromDegrees = function(longitude, latitude, height, ellipsoid, result) {
                Check.typeOf.number('longitude', longitude);
        Check.typeOf.number('latitude', latitude);
        
        longitude = CesiumMath.toRadians(longitude);
        latitude = CesiumMath.toRadians(latitude);
        return Cartesian3.fromRadians(longitude, latitude, height, ellipsoid, result);
    };

    var scratchN = new Cartesian3();
    var scratchK = new Cartesian3();
    var wgs84RadiiSquared = new Cartesian3(6378137.0 * 6378137.0, 6378137.0 * 6378137.0, 6356752.3142451793 * 6356752.3142451793);

    /**
     * Returns a Cartesian3 position from longitude and latitude values given in radians.
     *
     * @param {Number} longitude The longitude, in radians
     * @param {Number} latitude The latitude, in radians
     * @param {Number} [height=0.0] The height, in meters, above the ellipsoid.
     * @param {Ellipsoid} [ellipsoid=Ellipsoid.WGS84] The ellipsoid on which the position lies.
     * @param {Cartesian3} [result] The object onto which to store the result.
     * @returns {Cartesian3} The position
     *
     * @example
     * var position = Cesium.Cartesian3.fromRadians(-2.007, 0.645);
     */
    Cartesian3.fromRadians = function(longitude, latitude, height, ellipsoid, result) {
                Check.typeOf.number('longitude', longitude);
        Check.typeOf.number('latitude', latitude);
        
        height = defaultValue(height, 0.0);
        var radiiSquared = defined(ellipsoid) ? ellipsoid.radiiSquared : wgs84RadiiSquared;

        var cosLatitude = Math.cos(latitude);
        scratchN.x = cosLatitude * Math.cos(longitude);
        scratchN.y = cosLatitude * Math.sin(longitude);
        scratchN.z = Math.sin(latitude);
        scratchN = Cartesian3.normalize(scratchN, scratchN);

        Cartesian3.multiplyComponents(radiiSquared, scratchN, scratchK);
        var gamma = Math.sqrt(Cartesian3.dot(scratchN, scratchK));
        scratchK = Cartesian3.divideByScalar(scratchK, gamma, scratchK);
        scratchN = Cartesian3.multiplyByScalar(scratchN, height, scratchN);

        if (!defined(result)) {
            result = new Cartesian3();
        }
        return Cartesian3.add(scratchK, scratchN, result);
    };

    /**
     * Returns an array of Cartesian3 positions given an array of longitude and latitude values given in degrees.
     *
     * @param {Number[]} coordinates A list of longitude and latitude values. Values alternate [longitude, latitude, longitude, latitude...].
     * @param {Ellipsoid} [ellipsoid=Ellipsoid.WGS84] The ellipsoid on which the coordinates lie.
     * @param {Cartesian3[]} [result] An array of Cartesian3 objects to store the result.
     * @returns {Cartesian3[]} The array of positions.
     *
     * @example
     * var positions = Cesium.Cartesian3.fromDegreesArray([-115.0, 37.0, -107.0, 33.0]);
     */
    Cartesian3.fromDegreesArray = function(coordinates, ellipsoid, result) {
                Check.defined('coordinates', coordinates);
        if (coordinates.length < 2 || coordinates.length % 2 !== 0) {
            throw new DeveloperError('the number of coordinates must be a multiple of 2 and at least 2');
        }
        
        var length = coordinates.length;
        if (!defined(result)) {
            result = new Array(length / 2);
        } else {
            result.length = length / 2;
        }

        for (var i = 0; i < length; i += 2) {
            var longitude = coordinates[i];
            var latitude = coordinates[i + 1];
            var index = i / 2;
            result[index] = Cartesian3.fromDegrees(longitude, latitude, 0, ellipsoid, result[index]);
        }

        return result;
    };

    /**
     * Returns an array of Cartesian3 positions given an array of longitude and latitude values given in radians.
     *
     * @param {Number[]} coordinates A list of longitude and latitude values. Values alternate [longitude, latitude, longitude, latitude...].
     * @param {Ellipsoid} [ellipsoid=Ellipsoid.WGS84] The ellipsoid on which the coordinates lie.
     * @param {Cartesian3[]} [result] An array of Cartesian3 objects to store the result.
     * @returns {Cartesian3[]} The array of positions.
     *
     * @example
     * var positions = Cesium.Cartesian3.fromRadiansArray([-2.007, 0.645, -1.867, .575]);
     */
    Cartesian3.fromRadiansArray = function(coordinates, ellipsoid, result) {
                Check.defined('coordinates', coordinates);
        if (coordinates.length < 2 || coordinates.length % 2 !== 0) {
            throw new DeveloperError('the number of coordinates must be a multiple of 2 and at least 2');
        }
        
        var length = coordinates.length;
        if (!defined(result)) {
            result = new Array(length / 2);
        } else {
            result.length = length / 2;
        }

        for (var i = 0; i < length; i += 2) {
            var longitude = coordinates[i];
            var latitude = coordinates[i + 1];
            var index = i / 2;
            result[index] = Cartesian3.fromRadians(longitude, latitude, 0, ellipsoid, result[index]);
        }

        return result;
    };

    /**
     * Returns an array of Cartesian3 positions given an array of longitude, latitude and height values where longitude and latitude are given in degrees.
     *
     * @param {Number[]} coordinates A list of longitude, latitude and height values. Values alternate [longitude, latitude, height, longitude, latitude, height...].
     * @param {Ellipsoid} [ellipsoid=Ellipsoid.WGS84] The ellipsoid on which the position lies.
     * @param {Cartesian3[]} [result] An array of Cartesian3 objects to store the result.
     * @returns {Cartesian3[]} The array of positions.
     *
     * @example
     * var positions = Cesium.Cartesian3.fromDegreesArrayHeights([-115.0, 37.0, 100000.0, -107.0, 33.0, 150000.0]);
     */
    Cartesian3.fromDegreesArrayHeights = function(coordinates, ellipsoid, result) {
                Check.defined('coordinates', coordinates);
        if (coordinates.length < 3 || coordinates.length % 3 !== 0) {
            throw new DeveloperError('the number of coordinates must be a multiple of 3 and at least 3');
        }
        
        var length = coordinates.length;
        if (!defined(result)) {
            result = new Array(length / 3);
        } else {
            result.length = length / 3;
        }

        for (var i = 0; i < length; i += 3) {
            var longitude = coordinates[i];
            var latitude = coordinates[i + 1];
            var height = coordinates[i + 2];
            var index = i / 3;
            result[index] = Cartesian3.fromDegrees(longitude, latitude, height, ellipsoid, result[index]);
        }

        return result;
    };

    /**
     * Returns an array of Cartesian3 positions given an array of longitude, latitude and height values where longitude and latitude are given in radians.
     *
     * @param {Number[]} coordinates A list of longitude, latitude and height values. Values alternate [longitude, latitude, height, longitude, latitude, height...].
     * @param {Ellipsoid} [ellipsoid=Ellipsoid.WGS84] The ellipsoid on which the position lies.
     * @param {Cartesian3[]} [result] An array of Cartesian3 objects to store the result.
     * @returns {Cartesian3[]} The array of positions.
     *
     * @example
     * var positions = Cesium.Cartesian3.fromRadiansArrayHeights([-2.007, 0.645, 100000.0, -1.867, .575, 150000.0]);
     */
    Cartesian3.fromRadiansArrayHeights = function(coordinates, ellipsoid, result) {
                Check.defined('coordinates', coordinates);
        if (coordinates.length < 3 || coordinates.length % 3 !== 0) {
            throw new DeveloperError('the number of coordinates must be a multiple of 3 and at least 3');
        }
        
        var length = coordinates.length;
        if (!defined(result)) {
            result = new Array(length / 3);
        } else {
            result.length = length / 3;
        }

        for (var i = 0; i < length; i += 3) {
            var longitude = coordinates[i];
            var latitude = coordinates[i + 1];
            var height = coordinates[i + 2];
            var index = i / 3;
            result[index] = Cartesian3.fromRadians(longitude, latitude, height, ellipsoid, result[index]);
        }

        return result;
    };

    /**
     * An immutable Cartesian3 instance initialized to (0.0, 0.0, 0.0).
     *
     * @type {Cartesian3}
     * @constant
     */
    Cartesian3.ZERO = freezeObject(new Cartesian3(0.0, 0.0, 0.0));

    /**
     * An immutable Cartesian3 instance initialized to (1.0, 0.0, 0.0).
     *
     * @type {Cartesian3}
     * @constant
     */
    Cartesian3.UNIT_X = freezeObject(new Cartesian3(1.0, 0.0, 0.0));

    /**
     * An immutable Cartesian3 instance initialized to (0.0, 1.0, 0.0).
     *
     * @type {Cartesian3}
     * @constant
     */
    Cartesian3.UNIT_Y = freezeObject(new Cartesian3(0.0, 1.0, 0.0));

    /**
     * An immutable Cartesian3 instance initialized to (0.0, 0.0, 1.0).
     *
     * @type {Cartesian3}
     * @constant
     */
    Cartesian3.UNIT_Z = freezeObject(new Cartesian3(0.0, 0.0, 1.0));

    /**
     * Duplicates this Cartesian3 instance.
     *
     * @param {Cartesian3} [result] The object onto which to store the result.
     * @returns {Cartesian3} The modified result parameter or a new Cartesian3 instance if one was not provided.
     */
    Cartesian3.prototype.clone = function(result) {
        return Cartesian3.clone(this, result);
    };

    /**
     * Compares this Cartesian against the provided Cartesian componentwise and returns
     * <code>true</code> if they are equal, <code>false</code> otherwise.
     *
     * @param {Cartesian3} [right] The right hand side Cartesian.
     * @returns {Boolean} <code>true</code> if they are equal, <code>false</code> otherwise.
     */
    Cartesian3.prototype.equals = function(right) {
        return Cartesian3.equals(this, right);
    };

    /**
     * Compares this Cartesian against the provided Cartesian componentwise and returns
     * <code>true</code> if they pass an absolute or relative tolerance test,
     * <code>false</code> otherwise.
     *
     * @param {Cartesian3} [right] The right hand side Cartesian.
     * @param {Number} relativeEpsilon The relative epsilon tolerance to use for equality testing.
     * @param {Number} [absoluteEpsilon=relativeEpsilon] The absolute epsilon tolerance to use for equality testing.
     * @returns {Boolean} <code>true</code> if they are within the provided epsilon, <code>false</code> otherwise.
     */
    Cartesian3.prototype.equalsEpsilon = function(right, relativeEpsilon, absoluteEpsilon) {
        return Cartesian3.equalsEpsilon(this, right, relativeEpsilon, absoluteEpsilon);
    };

    /**
     * Creates a string representing this Cartesian in the format '(x, y, z)'.
     *
     * @returns {String} A string representing this Cartesian in the format '(x, y, z)'.
     */
    Cartesian3.prototype.toString = function() {
        return '(' + this.x + ', ' + this.y + ', ' + this.z + ')';
    };

    return Cartesian3;
});

define('Core/arrayFill',[
        './Check',
        './defaultValue',
        './defined'
    ], function(
        Check,
        defaultValue,
        defined) {
    'use strict';

    /**
     * Fill an array or a portion of an array with a given value.
     *
     * @param {Array} array The array to fill.
     * @param {*} value The value to fill the array with.
     * @param {Number} [start=0] The index to start filling at.
     * @param {Number} [end=array.length] The index to end stop at.
     *
     * @returns {Array} The resulting array.
     * @private
     */
    function arrayFill(array, value, start, end) {
                Check.defined('array', array);
        Check.defined('value', value);
        if (defined(start)) {
            Check.typeOf.number('start', start);
        }
        if (defined(end)) {
            Check.typeOf.number('end', end);
        }
        
        if (typeof array.fill === 'function') {
            return array.fill(value, start, end);
        }

        var length = array.length >>> 0;
        var relativeStart = defaultValue(start, 0);
        // If negative, find wrap around position
        var k = (relativeStart < 0) ? Math.max(length + relativeStart, 0) : Math.min(relativeStart, length);
        var relativeEnd = defaultValue(end, length);
        // If negative, find wrap around position
        var last = (relativeEnd < 0) ? Math.max(length + relativeEnd, 0) : Math.min(relativeEnd, length);

        // Fill array accordingly
        while (k < last) {
            array[k] = value;
            k++;
        }
        return array;
    }

    return arrayFill;
});

define('Core/scaleToGeodeticSurface',[
        './Cartesian3',
        './defined',
        './DeveloperError',
        './Math'
    ], function(
        Cartesian3,
        defined,
        DeveloperError,
        CesiumMath) {
    'use strict';

    var scaleToGeodeticSurfaceIntersection = new Cartesian3();
    var scaleToGeodeticSurfaceGradient = new Cartesian3();

    /**
     * Scales the provided Cartesian position along the geodetic surface normal
     * so that it is on the surface of this ellipsoid.  If the position is
     * at the center of the ellipsoid, this function returns undefined.
     *
     * @param {Cartesian3} cartesian The Cartesian position to scale.
     * @param {Cartesian3} oneOverRadii One over radii of the ellipsoid.
     * @param {Cartesian3} oneOverRadiiSquared One over radii squared of the ellipsoid.
     * @param {Number} centerToleranceSquared Tolerance for closeness to the center.
     * @param {Cartesian3} [result] The object onto which to store the result.
     * @returns {Cartesian3} The modified result parameter, a new Cartesian3 instance if none was provided, or undefined if the position is at the center.
     *
     * @exports scaleToGeodeticSurface
     *
     * @private
     */
    function scaleToGeodeticSurface(cartesian, oneOverRadii, oneOverRadiiSquared, centerToleranceSquared, result) {
                if (!defined(cartesian)) {
            throw new DeveloperError('cartesian is required.');
        }
        if (!defined(oneOverRadii)) {
            throw new DeveloperError('oneOverRadii is required.');
        }
        if (!defined(oneOverRadiiSquared)) {
            throw new DeveloperError('oneOverRadiiSquared is required.');
        }
        if (!defined(centerToleranceSquared)) {
            throw new DeveloperError('centerToleranceSquared is required.');
        }
        
        var positionX = cartesian.x;
        var positionY = cartesian.y;
        var positionZ = cartesian.z;

        var oneOverRadiiX = oneOverRadii.x;
        var oneOverRadiiY = oneOverRadii.y;
        var oneOverRadiiZ = oneOverRadii.z;

        var x2 = positionX * positionX * oneOverRadiiX * oneOverRadiiX;
        var y2 = positionY * positionY * oneOverRadiiY * oneOverRadiiY;
        var z2 = positionZ * positionZ * oneOverRadiiZ * oneOverRadiiZ;

        // Compute the squared ellipsoid norm.
        var squaredNorm = x2 + y2 + z2;
        var ratio = Math.sqrt(1.0 / squaredNorm);

        // As an initial approximation, assume that the radial intersection is the projection point.
        var intersection = Cartesian3.multiplyByScalar(cartesian, ratio, scaleToGeodeticSurfaceIntersection);

        // If the position is near the center, the iteration will not converge.
        if (squaredNorm < centerToleranceSquared) {
            return !isFinite(ratio) ? undefined : Cartesian3.clone(intersection, result);
        }

        var oneOverRadiiSquaredX = oneOverRadiiSquared.x;
        var oneOverRadiiSquaredY = oneOverRadiiSquared.y;
        var oneOverRadiiSquaredZ = oneOverRadiiSquared.z;

        // Use the gradient at the intersection point in place of the true unit normal.
        // The difference in magnitude will be absorbed in the multiplier.
        var gradient = scaleToGeodeticSurfaceGradient;
        gradient.x = intersection.x * oneOverRadiiSquaredX * 2.0;
        gradient.y = intersection.y * oneOverRadiiSquaredY * 2.0;
        gradient.z = intersection.z * oneOverRadiiSquaredZ * 2.0;

        // Compute the initial guess at the normal vector multiplier, lambda.
        var lambda = (1.0 - ratio) * Cartesian3.magnitude(cartesian) / (0.5 * Cartesian3.magnitude(gradient));
        var correction = 0.0;

        var func;
        var denominator;
        var xMultiplier;
        var yMultiplier;
        var zMultiplier;
        var xMultiplier2;
        var yMultiplier2;
        var zMultiplier2;
        var xMultiplier3;
        var yMultiplier3;
        var zMultiplier3;

        do {
            lambda -= correction;

            xMultiplier = 1.0 / (1.0 + lambda * oneOverRadiiSquaredX);
            yMultiplier = 1.0 / (1.0 + lambda * oneOverRadiiSquaredY);
            zMultiplier = 1.0 / (1.0 + lambda * oneOverRadiiSquaredZ);

            xMultiplier2 = xMultiplier * xMultiplier;
            yMultiplier2 = yMultiplier * yMultiplier;
            zMultiplier2 = zMultiplier * zMultiplier;

            xMultiplier3 = xMultiplier2 * xMultiplier;
            yMultiplier3 = yMultiplier2 * yMultiplier;
            zMultiplier3 = zMultiplier2 * zMultiplier;

            func = x2 * xMultiplier2 + y2 * yMultiplier2 + z2 * zMultiplier2 - 1.0;

            // "denominator" here refers to the use of this expression in the velocity and acceleration
            // computations in the sections to follow.
            denominator = x2 * xMultiplier3 * oneOverRadiiSquaredX + y2 * yMultiplier3 * oneOverRadiiSquaredY + z2 * zMultiplier3 * oneOverRadiiSquaredZ;

            var derivative = -2.0 * denominator;

            correction = func / derivative;
        } while (Math.abs(func) > CesiumMath.EPSILON12);

        if (!defined(result)) {
            return new Cartesian3(positionX * xMultiplier, positionY * yMultiplier, positionZ * zMultiplier);
        }
        result.x = positionX * xMultiplier;
        result.y = positionY * yMultiplier;
        result.z = positionZ * zMultiplier;
        return result;
    }

    return scaleToGeodeticSurface;
});

define('Core/Cartographic',[
        './Cartesian3',
        './Check',
        './defaultValue',
        './defined',
        './freezeObject',
        './Math',
        './scaleToGeodeticSurface'
    ], function(
        Cartesian3,
        Check,
        defaultValue,
        defined,
        freezeObject,
        CesiumMath,
        scaleToGeodeticSurface) {
    'use strict';

    /**
     * A position defined by longitude, latitude, and height.
     * @alias Cartographic
     * @constructor
     *
     * @param {Number} [longitude=0.0] The longitude, in radians.
     * @param {Number} [latitude=0.0] The latitude, in radians.
     * @param {Number} [height=0.0] The height, in meters, above the ellipsoid.
     *
     * @see Ellipsoid
     */
    function Cartographic(longitude, latitude, height) {
        /**
         * The longitude, in radians.
         * @type {Number}
         * @default 0.0
         */
        this.longitude = defaultValue(longitude, 0.0);

        /**
         * The latitude, in radians.
         * @type {Number}
         * @default 0.0
         */
        this.latitude = defaultValue(latitude, 0.0);

        /**
         * The height, in meters, above the ellipsoid.
         * @type {Number}
         * @default 0.0
         */
        this.height = defaultValue(height, 0.0);
    }

    /**
     * Creates a new Cartographic instance from longitude and latitude
     * specified in radians.
     *
     * @param {Number} longitude The longitude, in radians.
     * @param {Number} latitude The latitude, in radians.
     * @param {Number} [height=0.0] The height, in meters, above the ellipsoid.
     * @param {Cartographic} [result] The object onto which to store the result.
     * @returns {Cartographic} The modified result parameter or a new Cartographic instance if one was not provided.
     */
    Cartographic.fromRadians = function(longitude, latitude, height, result) {
                Check.typeOf.number('longitude', longitude);
        Check.typeOf.number('latitude', latitude);
        
        height = defaultValue(height, 0.0);

        if (!defined(result)) {
            return new Cartographic(longitude, latitude, height);
        }

        result.longitude = longitude;
        result.latitude = latitude;
        result.height = height;
        return result;
    };

    /**
     * Creates a new Cartographic instance from longitude and latitude
     * specified in degrees.  The values in the resulting object will
     * be in radians.
     *
     * @param {Number} longitude The longitude, in degrees.
     * @param {Number} latitude The latitude, in degrees.
     * @param {Number} [height=0.0] The height, in meters, above the ellipsoid.
     * @param {Cartographic} [result] The object onto which to store the result.
     * @returns {Cartographic} The modified result parameter or a new Cartographic instance if one was not provided.
     */
    Cartographic.fromDegrees = function(longitude, latitude, height, result) {
                Check.typeOf.number('longitude', longitude);
        Check.typeOf.number('latitude', latitude);
                longitude = CesiumMath.toRadians(longitude);
        latitude = CesiumMath.toRadians(latitude);

        return Cartographic.fromRadians(longitude, latitude, height, result);
    };

    var cartesianToCartographicN = new Cartesian3();
    var cartesianToCartographicP = new Cartesian3();
    var cartesianToCartographicH = new Cartesian3();
    var wgs84OneOverRadii = new Cartesian3(1.0 / 6378137.0, 1.0 / 6378137.0, 1.0 / 6356752.3142451793);
    var wgs84OneOverRadiiSquared = new Cartesian3(1.0 / (6378137.0 * 6378137.0), 1.0 / (6378137.0 * 6378137.0), 1.0 / (6356752.3142451793 * 6356752.3142451793));
    var wgs84CenterToleranceSquared = CesiumMath.EPSILON1;

    /**
     * Creates a new Cartographic instance from a Cartesian position. The values in the
     * resulting object will be in radians.
     *
     * @param {Cartesian3} cartesian The Cartesian position to convert to cartographic representation.
     * @param {Ellipsoid} [ellipsoid=Ellipsoid.WGS84] The ellipsoid on which the position lies.
     * @param {Cartographic} [result] The object onto which to store the result.
     * @returns {Cartographic} The modified result parameter, new Cartographic instance if none was provided, or undefined if the cartesian is at the center of the ellipsoid.
     */
    Cartographic.fromCartesian = function(cartesian, ellipsoid, result) {
        var oneOverRadii = defined(ellipsoid) ? ellipsoid.oneOverRadii : wgs84OneOverRadii;
        var oneOverRadiiSquared = defined(ellipsoid) ? ellipsoid.oneOverRadiiSquared : wgs84OneOverRadiiSquared;
        var centerToleranceSquared = defined(ellipsoid) ? ellipsoid._centerToleranceSquared : wgs84CenterToleranceSquared;

        //`cartesian is required.` is thrown from scaleToGeodeticSurface
        var p = scaleToGeodeticSurface(cartesian, oneOverRadii, oneOverRadiiSquared, centerToleranceSquared, cartesianToCartographicP);

        if (!defined(p)) {
            return undefined;
        }

        var n = Cartesian3.multiplyComponents(p, oneOverRadiiSquared, cartesianToCartographicN);
        n = Cartesian3.normalize(n, n);

        var h = Cartesian3.subtract(cartesian, p, cartesianToCartographicH);

        var longitude = Math.atan2(n.y, n.x);
        var latitude = Math.asin(n.z);
        var height = CesiumMath.sign(Cartesian3.dot(h, cartesian)) * Cartesian3.magnitude(h);

        if (!defined(result)) {
            return new Cartographic(longitude, latitude, height);
        }
        result.longitude = longitude;
        result.latitude = latitude;
        result.height = height;
        return result;
    };

    /**
     * Creates a new Cartesian3 instance from a Cartographic input. The values in the inputted
     * object should be in radians.
     *
     * @param {Cartographic} cartographic Input to be converted into a Cartesian3 output.
     * @param {Ellipsoid} [ellipsoid=Ellipsoid.WGS84] The ellipsoid on which the position lies.
     * @param {Cartesian3} [result] The object onto which to store the result.
     * @returns {Cartesian3} The position
     */
    Cartographic.toCartesian = function(cartographic, ellipsoid, result) {
                Check.defined('cartographic', cartographic);
        
        return Cartesian3.fromRadians(cartographic.longitude, cartographic.latitude, cartographic.height, ellipsoid, result);
    };

    /**
     * Duplicates a Cartographic instance.
     *
     * @param {Cartographic} cartographic The cartographic to duplicate.
     * @param {Cartographic} [result] The object onto which to store the result.
     * @returns {Cartographic} The modified result parameter or a new Cartographic instance if one was not provided. (Returns undefined if cartographic is undefined)
     */
    Cartographic.clone = function(cartographic, result) {
        if (!defined(cartographic)) {
            return undefined;
        }
        if (!defined(result)) {
            return new Cartographic(cartographic.longitude, cartographic.latitude, cartographic.height);
        }
        result.longitude = cartographic.longitude;
        result.latitude = cartographic.latitude;
        result.height = cartographic.height;
        return result;
    };

    /**
     * Compares the provided cartographics componentwise and returns
     * <code>true</code> if they are equal, <code>false</code> otherwise.
     *
     * @param {Cartographic} [left] The first cartographic.
     * @param {Cartographic} [right] The second cartographic.
     * @returns {Boolean} <code>true</code> if left and right are equal, <code>false</code> otherwise.
     */
    Cartographic.equals = function(left, right) {
        return (left === right) ||
                ((defined(left)) &&
                 (defined(right)) &&
                 (left.longitude === right.longitude) &&
                 (left.latitude === right.latitude) &&
                 (left.height === right.height));
    };

    /**
     * Compares the provided cartographics componentwise and returns
     * <code>true</code> if they are within the provided epsilon,
     * <code>false</code> otherwise.
     *
     * @param {Cartographic} [left] The first cartographic.
     * @param {Cartographic} [right] The second cartographic.
     * @param {Number} epsilon The epsilon to use for equality testing.
     * @returns {Boolean} <code>true</code> if left and right are within the provided epsilon, <code>false</code> otherwise.
     */
    Cartographic.equalsEpsilon = function(left, right, epsilon) {
                Check.typeOf.number('epsilon', epsilon);
        
        return (left === right) ||
               ((defined(left)) &&
                (defined(right)) &&
                (Math.abs(left.longitude - right.longitude) <= epsilon) &&
                (Math.abs(left.latitude - right.latitude) <= epsilon) &&
                (Math.abs(left.height - right.height) <= epsilon));
    };

    /**
     * An immutable Cartographic instance initialized to (0.0, 0.0, 0.0).
     *
     * @type {Cartographic}
     * @constant
     */
    Cartographic.ZERO = freezeObject(new Cartographic(0.0, 0.0, 0.0));

    /**
     * Duplicates this instance.
     *
     * @param {Cartographic} [result] The object onto which to store the result.
     * @returns {Cartographic} The modified result parameter or a new Cartographic instance if one was not provided.
     */
    Cartographic.prototype.clone = function(result) {
        return Cartographic.clone(this, result);
    };

    /**
     * Compares the provided against this cartographic componentwise and returns
     * <code>true</code> if they are equal, <code>false</code> otherwise.
     *
     * @param {Cartographic} [right] The second cartographic.
     * @returns {Boolean} <code>true</code> if left and right are equal, <code>false</code> otherwise.
     */
    Cartographic.prototype.equals = function(right) {
        return Cartographic.equals(this, right);
    };

    /**
     * Compares the provided against this cartographic componentwise and returns
     * <code>true</code> if they are within the provided epsilon,
     * <code>false</code> otherwise.
     *
     * @param {Cartographic} [right] The second cartographic.
     * @param {Number} epsilon The epsilon to use for equality testing.
     * @returns {Boolean} <code>true</code> if left and right are within the provided epsilon, <code>false</code> otherwise.
     */
    Cartographic.prototype.equalsEpsilon = function(right, epsilon) {
        return Cartographic.equalsEpsilon(this, right, epsilon);
    };

    /**
     * Creates a string representing this cartographic in the format '(longitude, latitude, height)'.
     *
     * @returns {String} A string representing the provided cartographic in the format '(longitude, latitude, height)'.
     */
    Cartographic.prototype.toString = function() {
        return '(' + this.longitude + ', ' + this.latitude + ', ' + this.height + ')';
    };

    return Cartographic;
});

define('Core/defineProperties',[
        './defined'
    ], function(
        defined) {
    'use strict';

    var definePropertyWorks = (function() {
        try {
            return 'x' in Object.defineProperty({}, 'x', {});
        } catch (e) {
            return false;
        }
    })();

    /**
     * Defines properties on an object, using Object.defineProperties if available,
     * otherwise returns the object unchanged.  This function should be used in
     * setup code to prevent errors from completely halting JavaScript execution
     * in legacy browsers.
     *
     * @private
     *
     * @exports defineProperties
     */
    var defineProperties = Object.defineProperties;
    if (!definePropertyWorks || !defined(defineProperties)) {
        defineProperties = function(o) {
            return o;
        };
    }

    return defineProperties;
});

define('Core/Ellipsoid',[
        './Cartesian3',
        './Cartographic',
        './Check',
        './defaultValue',
        './defined',
        './defineProperties',
        './DeveloperError',
        './freezeObject',
        './Math',
        './scaleToGeodeticSurface'
    ], function(
        Cartesian3,
        Cartographic,
        Check,
        defaultValue,
        defined,
        defineProperties,
        DeveloperError,
        freezeObject,
        CesiumMath,
        scaleToGeodeticSurface) {
    'use strict';

    function initialize(ellipsoid, x, y, z) {
        x = defaultValue(x, 0.0);
        y = defaultValue(y, 0.0);
        z = defaultValue(z, 0.0);

                Check.typeOf.number.greaterThanOrEquals('x', x, 0.0);
        Check.typeOf.number.greaterThanOrEquals('y', y, 0.0);
        Check.typeOf.number.greaterThanOrEquals('z', z, 0.0);
        
        ellipsoid._radii = new Cartesian3(x, y, z);

        ellipsoid._radiiSquared = new Cartesian3(x * x,
                                            y * y,
                                            z * z);

        ellipsoid._radiiToTheFourth = new Cartesian3(x * x * x * x,
                                                y * y * y * y,
                                                z * z * z * z);

        ellipsoid._oneOverRadii = new Cartesian3(x === 0.0 ? 0.0 : 1.0 / x,
                                            y === 0.0 ? 0.0 : 1.0 / y,
                                            z === 0.0 ? 0.0 : 1.0 / z);

        ellipsoid._oneOverRadiiSquared = new Cartesian3(x === 0.0 ? 0.0 : 1.0 / (x * x),
                                                   y === 0.0 ? 0.0 : 1.0 / (y * y),
                                                   z === 0.0 ? 0.0 : 1.0 / (z * z));

        ellipsoid._minimumRadius = Math.min(x, y, z);

        ellipsoid._maximumRadius = Math.max(x, y, z);

        ellipsoid._centerToleranceSquared = CesiumMath.EPSILON1;

        if (ellipsoid._radiiSquared.z !== 0) {
            ellipsoid._squaredXOverSquaredZ = ellipsoid._radiiSquared.x / ellipsoid._radiiSquared.z;
        }
    }

    /**
     * A quadratic surface defined in Cartesian coordinates by the equation
     * <code>(x / a)^2 + (y / b)^2 + (z / c)^2 = 1</code>.  Primarily used
     * by Cesium to represent the shape of planetary bodies.
     *
     * Rather than constructing this object directly, one of the provided
     * constants is normally used.
     * @alias Ellipsoid
     * @constructor
     *
     * @param {Number} [x=0] The radius in the x direction.
     * @param {Number} [y=0] The radius in the y direction.
     * @param {Number} [z=0] The radius in the z direction.
     *
     * @exception {DeveloperError} All radii components must be greater than or equal to zero.
     *
     * @see Ellipsoid.fromCartesian3
     * @see Ellipsoid.WGS84
     * @see Ellipsoid.UNIT_SPHERE
     */
    function Ellipsoid(x, y, z) {
        this._radii = undefined;
        this._radiiSquared = undefined;
        this._radiiToTheFourth = undefined;
        this._oneOverRadii = undefined;
        this._oneOverRadiiSquared = undefined;
        this._minimumRadius = undefined;
        this._maximumRadius = undefined;
        this._centerToleranceSquared = undefined;
        this._squaredXOverSquaredZ = undefined;

        initialize(this, x, y, z);
    }

    defineProperties(Ellipsoid.prototype, {
        /**
         * Gets the radii of the ellipsoid.
         * @memberof Ellipsoid.prototype
         * @type {Cartesian3}
         * @readonly
         */
        radii : {
            get: function() {
                return this._radii;
            }
        },
        /**
         * Gets the squared radii of the ellipsoid.
         * @memberof Ellipsoid.prototype
         * @type {Cartesian3}
         * @readonly
         */
        radiiSquared : {
            get : function() {
                return this._radiiSquared;
            }
        },
        /**
         * Gets the radii of the ellipsoid raise to the fourth power.
         * @memberof Ellipsoid.prototype
         * @type {Cartesian3}
         * @readonly
         */
        radiiToTheFourth : {
            get : function() {
                return this._radiiToTheFourth;
            }
        },
        /**
         * Gets one over the radii of the ellipsoid.
         * @memberof Ellipsoid.prototype
         * @type {Cartesian3}
         * @readonly
         */
        oneOverRadii : {
            get : function() {
                return this._oneOverRadii;
            }
        },
        /**
         * Gets one over the squared radii of the ellipsoid.
         * @memberof Ellipsoid.prototype
         * @type {Cartesian3}
         * @readonly
         */
        oneOverRadiiSquared : {
            get : function() {
                return this._oneOverRadiiSquared;
            }
        },
        /**
         * Gets the minimum radius of the ellipsoid.
         * @memberof Ellipsoid.prototype
         * @type {Number}
         * @readonly
         */
        minimumRadius : {
            get : function() {
                return this._minimumRadius;
            }
        },
        /**
         * Gets the maximum radius of the ellipsoid.
         * @memberof Ellipsoid.prototype
         * @type {Number}
         * @readonly
         */
        maximumRadius : {
            get : function() {
                return this._maximumRadius;
            }
        }
    });

    /**
     * Duplicates an Ellipsoid instance.
     *
     * @param {Ellipsoid} ellipsoid The ellipsoid to duplicate.
     * @param {Ellipsoid} [result] The object onto which to store the result, or undefined if a new
     *                    instance should be created.
     * @returns {Ellipsoid} The cloned Ellipsoid. (Returns undefined if ellipsoid is undefined)
     */
    Ellipsoid.clone = function(ellipsoid, result) {
        if (!defined(ellipsoid)) {
            return undefined;
        }
        var radii = ellipsoid._radii;

        if (!defined(result)) {
            return new Ellipsoid(radii.x, radii.y, radii.z);
        }

        Cartesian3.clone(radii, result._radii);
        Cartesian3.clone(ellipsoid._radiiSquared, result._radiiSquared);
        Cartesian3.clone(ellipsoid._radiiToTheFourth, result._radiiToTheFourth);
        Cartesian3.clone(ellipsoid._oneOverRadii, result._oneOverRadii);
        Cartesian3.clone(ellipsoid._oneOverRadiiSquared, result._oneOverRadiiSquared);
        result._minimumRadius = ellipsoid._minimumRadius;
        result._maximumRadius = ellipsoid._maximumRadius;
        result._centerToleranceSquared = ellipsoid._centerToleranceSquared;

        return result;
    };

    /**
     * Computes an Ellipsoid from a Cartesian specifying the radii in x, y, and z directions.
     *
     * @param {Cartesian3} [cartesian=Cartesian3.ZERO] The ellipsoid's radius in the x, y, and z directions.
     * @param {Ellipsoid} [result] The object onto which to store the result, or undefined if a new
     *                    instance should be created.
     * @returns {Ellipsoid} A new Ellipsoid instance.
     *
     * @exception {DeveloperError} All radii components must be greater than or equal to zero.
     *
     * @see Ellipsoid.WGS84
     * @see Ellipsoid.UNIT_SPHERE
     */
    Ellipsoid.fromCartesian3 = function(cartesian, result) {
        if (!defined(result)) {
            result = new Ellipsoid();
        }

        if (!defined(cartesian)) {
            return result;
        }

        initialize(result, cartesian.x, cartesian.y, cartesian.z);
        return result;
    };

    /**
     * An Ellipsoid instance initialized to the WGS84 standard.
     *
     * @type {Ellipsoid}
     * @constant
     */
    Ellipsoid.WGS84 = freezeObject(new Ellipsoid(6378137.0, 6378137.0, 6356752.3142451793));

    /**
     * An Ellipsoid instance initialized to radii of (1.0, 1.0, 1.0).
     *
     * @type {Ellipsoid}
     * @constant
     */
    Ellipsoid.UNIT_SPHERE = freezeObject(new Ellipsoid(1.0, 1.0, 1.0));

    /**
     * An Ellipsoid instance initialized to a sphere with the lunar radius.
     *
     * @type {Ellipsoid}
     * @constant
     */
    Ellipsoid.MOON = freezeObject(new Ellipsoid(CesiumMath.LUNAR_RADIUS, CesiumMath.LUNAR_RADIUS, CesiumMath.LUNAR_RADIUS));

    /**
     * Duplicates an Ellipsoid instance.
     *
     * @param {Ellipsoid} [result] The object onto which to store the result, or undefined if a new
     *                    instance should be created.
     * @returns {Ellipsoid} The cloned Ellipsoid.
     */
    Ellipsoid.prototype.clone = function(result) {
        return Ellipsoid.clone(this, result);
    };

    /**
     * The number of elements used to pack the object into an array.
     * @type {Number}
     */
    Ellipsoid.packedLength = Cartesian3.packedLength;

    /**
     * Stores the provided instance into the provided array.
     *
     * @param {Ellipsoid} value The value to pack.
     * @param {Number[]} array The array to pack into.
     * @param {Number} [startingIndex=0] The index into the array at which to start packing the elements.
     *
     * @returns {Number[]} The array that was packed into
     */
    Ellipsoid.pack = function(value, array, startingIndex) {
                Check.typeOf.object('value', value);
        Check.defined('array', array);
        
        startingIndex = defaultValue(startingIndex, 0);

        Cartesian3.pack(value._radii, array, startingIndex);

        return array;
    };

    /**
     * Retrieves an instance from a packed array.
     *
     * @param {Number[]} array The packed array.
     * @param {Number} [startingIndex=0] The starting index of the element to be unpacked.
     * @param {Ellipsoid} [result] The object into which to store the result.
     * @returns {Ellipsoid} The modified result parameter or a new Ellipsoid instance if one was not provided.
     */
    Ellipsoid.unpack = function(array, startingIndex, result) {
                Check.defined('array', array);
        
        startingIndex = defaultValue(startingIndex, 0);

        var radii = Cartesian3.unpack(array, startingIndex);
        return Ellipsoid.fromCartesian3(radii, result);
    };

    /**
     * Computes the unit vector directed from the center of this ellipsoid toward the provided Cartesian position.
     * @function
     *
     * @param {Cartesian3} cartesian The Cartesian for which to to determine the geocentric normal.
     * @param {Cartesian3} [result] The object onto which to store the result.
     * @returns {Cartesian3} The modified result parameter or a new Cartesian3 instance if none was provided.
     */
    Ellipsoid.prototype.geocentricSurfaceNormal = Cartesian3.normalize;

    /**
     * Computes the normal of the plane tangent to the surface of the ellipsoid at the provided position.
     *
     * @param {Cartographic} cartographic The cartographic position for which to to determine the geodetic normal.
     * @param {Cartesian3} [result] The object onto which to store the result.
     * @returns {Cartesian3} The modified result parameter or a new Cartesian3 instance if none was provided.
     */
    Ellipsoid.prototype.geodeticSurfaceNormalCartographic = function(cartographic, result) {
                Check.typeOf.object('cartographic', cartographic);
        
        var longitude = cartographic.longitude;
        var latitude = cartographic.latitude;
        var cosLatitude = Math.cos(latitude);

        var x = cosLatitude * Math.cos(longitude);
        var y = cosLatitude * Math.sin(longitude);
        var z = Math.sin(latitude);

        if (!defined(result)) {
            result = new Cartesian3();
        }
        result.x = x;
        result.y = y;
        result.z = z;
        return Cartesian3.normalize(result, result);
    };

    /**
     * Computes the normal of the plane tangent to the surface of the ellipsoid at the provided position.
     *
     * @param {Cartesian3} cartesian The Cartesian position for which to to determine the surface normal.
     * @param {Cartesian3} [result] The object onto which to store the result.
     * @returns {Cartesian3} The modified result parameter or a new Cartesian3 instance if none was provided.
     */
    Ellipsoid.prototype.geodeticSurfaceNormal = function(cartesian, result) {
        if (!defined(result)) {
            result = new Cartesian3();
        }
        result = Cartesian3.multiplyComponents(cartesian, this._oneOverRadiiSquared, result);
        return Cartesian3.normalize(result, result);
    };

    var cartographicToCartesianNormal = new Cartesian3();
    var cartographicToCartesianK = new Cartesian3();

    /**
     * Converts the provided cartographic to Cartesian representation.
     *
     * @param {Cartographic} cartographic The cartographic position.
     * @param {Cartesian3} [result] The object onto which to store the result.
     * @returns {Cartesian3} The modified result parameter or a new Cartesian3 instance if none was provided.
     *
     * @example
     * //Create a Cartographic and determine it's Cartesian representation on a WGS84 ellipsoid.
     * var position = new Cesium.Cartographic(Cesium.Math.toRadians(21), Cesium.Math.toRadians(78), 5000);
     * var cartesianPosition = Cesium.Ellipsoid.WGS84.cartographicToCartesian(position);
     */
    Ellipsoid.prototype.cartographicToCartesian = function(cartographic, result) {
        //`cartographic is required` is thrown from geodeticSurfaceNormalCartographic.
        var n = cartographicToCartesianNormal;
        var k = cartographicToCartesianK;
        this.geodeticSurfaceNormalCartographic(cartographic, n);
        Cartesian3.multiplyComponents(this._radiiSquared, n, k);
        var gamma = Math.sqrt(Cartesian3.dot(n, k));
        Cartesian3.divideByScalar(k, gamma, k);
        Cartesian3.multiplyByScalar(n, cartographic.height, n);

        if (!defined(result)) {
            result = new Cartesian3();
        }
        return Cartesian3.add(k, n, result);
    };

    /**
     * Converts the provided array of cartographics to an array of Cartesians.
     *
     * @param {Cartographic[]} cartographics An array of cartographic positions.
     * @param {Cartesian3[]} [result] The object onto which to store the result.
     * @returns {Cartesian3[]} The modified result parameter or a new Array instance if none was provided.
     *
     * @example
     * //Convert an array of Cartographics and determine their Cartesian representation on a WGS84 ellipsoid.
     * var positions = [new Cesium.Cartographic(Cesium.Math.toRadians(21), Cesium.Math.toRadians(78), 0),
     *                  new Cesium.Cartographic(Cesium.Math.toRadians(21.321), Cesium.Math.toRadians(78.123), 100),
     *                  new Cesium.Cartographic(Cesium.Math.toRadians(21.645), Cesium.Math.toRadians(78.456), 250)];
     * var cartesianPositions = Cesium.Ellipsoid.WGS84.cartographicArrayToCartesianArray(positions);
     */
    Ellipsoid.prototype.cartographicArrayToCartesianArray = function(cartographics, result) {
                Check.defined('cartographics', cartographics);
        
        var length = cartographics.length;
        if (!defined(result)) {
            result = new Array(length);
        } else {
            result.length = length;
        }
        for ( var i = 0; i < length; i++) {
            result[i] = this.cartographicToCartesian(cartographics[i], result[i]);
        }
        return result;
    };

    var cartesianToCartographicN = new Cartesian3();
    var cartesianToCartographicP = new Cartesian3();
    var cartesianToCartographicH = new Cartesian3();

    /**
     * Converts the provided cartesian to cartographic representation.
     * The cartesian is undefined at the center of the ellipsoid.
     *
     * @param {Cartesian3} cartesian The Cartesian position to convert to cartographic representation.
     * @param {Cartographic} [result] The object onto which to store the result.
     * @returns {Cartographic} The modified result parameter, new Cartographic instance if none was provided, or undefined if the cartesian is at the center of the ellipsoid.
     *
     * @example
     * //Create a Cartesian and determine it's Cartographic representation on a WGS84 ellipsoid.
     * var position = new Cesium.Cartesian3(17832.12, 83234.52, 952313.73);
     * var cartographicPosition = Cesium.Ellipsoid.WGS84.cartesianToCartographic(position);
     */
    Ellipsoid.prototype.cartesianToCartographic = function(cartesian, result) {
        //`cartesian is required.` is thrown from scaleToGeodeticSurface
        var p = this.scaleToGeodeticSurface(cartesian, cartesianToCartographicP);

        if (!defined(p)) {
            return undefined;
        }

        var n = this.geodeticSurfaceNormal(p, cartesianToCartographicN);
        var h = Cartesian3.subtract(cartesian, p, cartesianToCartographicH);

        var longitude = Math.atan2(n.y, n.x);
        var latitude = Math.asin(n.z);
        var height = CesiumMath.sign(Cartesian3.dot(h, cartesian)) * Cartesian3.magnitude(h);

        if (!defined(result)) {
            return new Cartographic(longitude, latitude, height);
        }
        result.longitude = longitude;
        result.latitude = latitude;
        result.height = height;
        return result;
    };

    /**
     * Converts the provided array of cartesians to an array of cartographics.
     *
     * @param {Cartesian3[]} cartesians An array of Cartesian positions.
     * @param {Cartographic[]} [result] The object onto which to store the result.
     * @returns {Cartographic[]} The modified result parameter or a new Array instance if none was provided.
     *
     * @example
     * //Create an array of Cartesians and determine their Cartographic representation on a WGS84 ellipsoid.
     * var positions = [new Cesium.Cartesian3(17832.12, 83234.52, 952313.73),
     *                  new Cesium.Cartesian3(17832.13, 83234.53, 952313.73),
     *                  new Cesium.Cartesian3(17832.14, 83234.54, 952313.73)]
     * var cartographicPositions = Cesium.Ellipsoid.WGS84.cartesianArrayToCartographicArray(positions);
     */
    Ellipsoid.prototype.cartesianArrayToCartographicArray = function(cartesians, result) {
                Check.defined('cartesians', cartesians);
        
        var length = cartesians.length;
        if (!defined(result)) {
            result = new Array(length);
        } else {
            result.length = length;
        }
        for ( var i = 0; i < length; ++i) {
            result[i] = this.cartesianToCartographic(cartesians[i], result[i]);
        }
        return result;
    };

    /**
     * Scales the provided Cartesian position along the geodetic surface normal
     * so that it is on the surface of this ellipsoid.  If the position is
     * at the center of the ellipsoid, this function returns undefined.
     *
     * @param {Cartesian3} cartesian The Cartesian position to scale.
     * @param {Cartesian3} [result] The object onto which to store the result.
     * @returns {Cartesian3} The modified result parameter, a new Cartesian3 instance if none was provided, or undefined if the position is at the center.
     */
    Ellipsoid.prototype.scaleToGeodeticSurface = function(cartesian, result) {
        return scaleToGeodeticSurface(cartesian, this._oneOverRadii, this._oneOverRadiiSquared, this._centerToleranceSquared, result);
    };

    /**
     * Scales the provided Cartesian position along the geocentric surface normal
     * so that it is on the surface of this ellipsoid.
     *
     * @param {Cartesian3} cartesian The Cartesian position to scale.
     * @param {Cartesian3} [result] The object onto which to store the result.
     * @returns {Cartesian3} The modified result parameter or a new Cartesian3 instance if none was provided.
     */
    Ellipsoid.prototype.scaleToGeocentricSurface = function(cartesian, result) {
                Check.typeOf.object('cartesian', cartesian);
        
        if (!defined(result)) {
            result = new Cartesian3();
        }

        var positionX = cartesian.x;
        var positionY = cartesian.y;
        var positionZ = cartesian.z;
        var oneOverRadiiSquared = this._oneOverRadiiSquared;

        var beta = 1.0 / Math.sqrt((positionX * positionX) * oneOverRadiiSquared.x +
                                   (positionY * positionY) * oneOverRadiiSquared.y +
                                   (positionZ * positionZ) * oneOverRadiiSquared.z);

        return Cartesian3.multiplyByScalar(cartesian, beta, result);
    };

    /**
     * Transforms a Cartesian X, Y, Z position to the ellipsoid-scaled space by multiplying
     * its components by the result of {@link Ellipsoid#oneOverRadii}.
     *
     * @param {Cartesian3} position The position to transform.
     * @param {Cartesian3} [result] The position to which to copy the result, or undefined to create and
     *        return a new instance.
     * @returns {Cartesian3} The position expressed in the scaled space.  The returned instance is the
     *          one passed as the result parameter if it is not undefined, or a new instance of it is.
     */
    Ellipsoid.prototype.transformPositionToScaledSpace = function(position, result) {
        if (!defined(result)) {
            result = new Cartesian3();
        }

        return Cartesian3.multiplyComponents(position, this._oneOverRadii, result);
    };

    /**
     * Transforms a Cartesian X, Y, Z position from the ellipsoid-scaled space by multiplying
     * its components by the result of {@link Ellipsoid#radii}.
     *
     * @param {Cartesian3} position The position to transform.
     * @param {Cartesian3} [result] The position to which to copy the result, or undefined to create and
     *        return a new instance.
     * @returns {Cartesian3} The position expressed in the unscaled space.  The returned instance is the
     *          one passed as the result parameter if it is not undefined, or a new instance of it is.
     */
    Ellipsoid.prototype.transformPositionFromScaledSpace = function(position, result) {
        if (!defined(result)) {
            result = new Cartesian3();
        }

        return Cartesian3.multiplyComponents(position, this._radii, result);
    };

    /**
     * Compares this Ellipsoid against the provided Ellipsoid componentwise and returns
     * <code>true</code> if they are equal, <code>false</code> otherwise.
     *
     * @param {Ellipsoid} [right] The other Ellipsoid.
     * @returns {Boolean} <code>true</code> if they are equal, <code>false</code> otherwise.
     */
    Ellipsoid.prototype.equals = function(right) {
        return (this === right) ||
               (defined(right) &&
                Cartesian3.equals(this._radii, right._radii));
    };

    /**
     * Creates a string representing this Ellipsoid in the format '(radii.x, radii.y, radii.z)'.
     *
     * @returns {String} A string representing this ellipsoid in the format '(radii.x, radii.y, radii.z)'.
     */
    Ellipsoid.prototype.toString = function() {
        return this._radii.toString();
    };

    /**
     * Computes a point which is the intersection of the surface normal with the z-axis.
     *
     * @param {Cartesian3} position the position. must be on the surface of the ellipsoid.
     * @param {Number} [buffer = 0.0] A buffer to subtract from the ellipsoid size when checking if the point is inside the ellipsoid.
     *                                In earth case, with common earth datums, there is no need for this buffer since the intersection point is always (relatively) very close to the center.
     *                                In WGS84 datum, intersection point is at max z = +-42841.31151331382 (0.673% of z-axis).
     *                                Intersection point could be outside the ellipsoid if the ratio of MajorAxis / AxisOfRotation is bigger than the square root of 2
     * @param {Cartesian3} [result] The cartesian to which to copy the result, or undefined to create and
     *        return a new instance.
     * @returns {Cartesian3 | undefined} the intersection point if it's inside the ellipsoid, undefined otherwise
     *
     * @exception {DeveloperError} position is required.
     * @exception {DeveloperError} Ellipsoid must be an ellipsoid of revolution (radii.x == radii.y).
     * @exception {DeveloperError} Ellipsoid.radii.z must be greater than 0.
     */
    Ellipsoid.prototype.getSurfaceNormalIntersectionWithZAxis = function(position, buffer, result) {
                Check.typeOf.object('position', position);

        if (!CesiumMath.equalsEpsilon(this._radii.x, this._radii.y, CesiumMath.EPSILON15)) {
            throw new DeveloperError('Ellipsoid must be an ellipsoid of revolution (radii.x == radii.y)');
        }

        Check.typeOf.number.greaterThan('Ellipsoid.radii.z', this._radii.z, 0);
        
        buffer = defaultValue(buffer, 0.0);

        var squaredXOverSquaredZ = this._squaredXOverSquaredZ;

        if (!defined(result)) {
            result = new Cartesian3();
        }

        result.x = 0.0;
        result.y = 0.0;
        result.z = position.z * (1 - squaredXOverSquaredZ);

        if (Math.abs(result.z) >= this._radii.z - buffer) {
            return undefined;
        }

        return result;
    };

    return Ellipsoid;
});

define('Core/GeographicProjection',[
        './Cartesian3',
        './Cartographic',
        './defaultValue',
        './defined',
        './defineProperties',
        './DeveloperError',
        './Ellipsoid'
    ], function(
        Cartesian3,
        Cartographic,
        defaultValue,
        defined,
        defineProperties,
        DeveloperError,
        Ellipsoid) {
    'use strict';

    /**
     * A simple map projection where longitude and latitude are linearly mapped to X and Y by multiplying
     * them by the {@link Ellipsoid#maximumRadius}.  This projection
     * is commonly known as geographic, equirectangular, equidistant cylindrical, or plate carrée.  It
     * is also known as EPSG:4326.
     *
     * @alias GeographicProjection
     * @constructor
     *
     * @param {Ellipsoid} [ellipsoid=Ellipsoid.WGS84] The ellipsoid.
     *
     * @see WebMercatorProjection
     */
    function GeographicProjection(ellipsoid) {
        this._ellipsoid = defaultValue(ellipsoid, Ellipsoid.WGS84);
        this._semimajorAxis = this._ellipsoid.maximumRadius;
        this._oneOverSemimajorAxis = 1.0 / this._semimajorAxis;
    }

    defineProperties(GeographicProjection.prototype, {
        /**
         * Gets the {@link Ellipsoid}.
         *
         * @memberof GeographicProjection.prototype
         *
         * @type {Ellipsoid}
         * @readonly
         */
        ellipsoid : {
            get : function() {
                return this._ellipsoid;
            }
        }
    });

    /**
     * Projects a set of {@link Cartographic} coordinates, in radians, to map coordinates, in meters.
     * X and Y are the longitude and latitude, respectively, multiplied by the maximum radius of the
     * ellipsoid.  Z is the unmodified height.
     *
     * @param {Cartographic} cartographic The coordinates to project.
     * @param {Cartesian3} [result] An instance into which to copy the result.  If this parameter is
     *        undefined, a new instance is created and returned.
     * @returns {Cartesian3} The projected coordinates.  If the result parameter is not undefined, the
     *          coordinates are copied there and that instance is returned.  Otherwise, a new instance is
     *          created and returned.
     */
    GeographicProjection.prototype.project = function(cartographic, result) {
        // Actually this is the special case of equidistant cylindrical called the plate carree
        var semimajorAxis = this._semimajorAxis;
        var x = cartographic.longitude * semimajorAxis;
        var y = cartographic.latitude * semimajorAxis;
        var z = cartographic.height;

        if (!defined(result)) {
            return new Cartesian3(x, y, z);
        }

        result.x = x;
        result.y = y;
        result.z = z;
        return result;
    };

    /**
     * Unprojects a set of projected {@link Cartesian3} coordinates, in meters, to {@link Cartographic}
     * coordinates, in radians.  Longitude and Latitude are the X and Y coordinates, respectively,
     * divided by the maximum radius of the ellipsoid.  Height is the unmodified Z coordinate.
     *
     * @param {Cartesian3} cartesian The Cartesian position to unproject with height (z) in meters.
     * @param {Cartographic} [result] An instance into which to copy the result.  If this parameter is
     *        undefined, a new instance is created and returned.
     * @returns {Cartographic} The unprojected coordinates.  If the result parameter is not undefined, the
     *          coordinates are copied there and that instance is returned.  Otherwise, a new instance is
     *          created and returned.
     */
    GeographicProjection.prototype.unproject = function(cartesian, result) {
                if (!defined(cartesian)) {
            throw new DeveloperError('cartesian is required');
        }
        
        var oneOverEarthSemimajorAxis = this._oneOverSemimajorAxis;
        var longitude = cartesian.x * oneOverEarthSemimajorAxis;
        var latitude = cartesian.y * oneOverEarthSemimajorAxis;
        var height = cartesian.z;

        if (!defined(result)) {
            return new Cartographic(longitude, latitude, height);
        }

        result.longitude = longitude;
        result.latitude = latitude;
        result.height = height;
        return result;
    };

    return GeographicProjection;
});

define('Core/Intersect',[
        './freezeObject'
    ], function(
        freezeObject) {
    'use strict';

    /**
     * This enumerated type is used in determining where, relative to the frustum, an
     * object is located. The object can either be fully contained within the frustum (INSIDE),
     * partially inside the frustum and partially outside (INTERSECTING), or somwhere entirely
     * outside of the frustum's 6 planes (OUTSIDE).
     *
     * @exports Intersect
     */
    var Intersect = {
        /**
         * Represents that an object is not contained within the frustum.
         *
         * @type {Number}
         * @constant
         */
        OUTSIDE : -1,

        /**
         * Represents that an object intersects one of the frustum's planes.
         *
         * @type {Number}
         * @constant
         */
        INTERSECTING : 0,

        /**
         * Represents that an object is fully within the frustum.
         *
         * @type {Number}
         * @constant
         */
        INSIDE : 1
    };

    return freezeObject(Intersect);
});

define('Core/Interval',[
        './defaultValue'
    ], function(
        defaultValue) {
    'use strict';

    /**
     * Represents the closed interval [start, stop].
     * @alias Interval
     * @constructor
     *
     * @param {Number} [start=0.0] The beginning of the interval.
     * @param {Number} [stop=0.0] The end of the interval.
     */
    function Interval(start, stop) {
        /**
         * The beginning of the interval.
         * @type {Number}
         * @default 0.0
         */
        this.start = defaultValue(start, 0.0);
        /**
         * The end of the interval.
         * @type {Number}
         * @default 0.0
         */
        this.stop = defaultValue(stop, 0.0);
    }

    return Interval;
});

define('Core/Matrix3',[
        './Cartesian3',
        './Check',
        './defaultValue',
        './defined',
        './defineProperties',
        './DeveloperError',
        './freezeObject',
        './Math'
    ], function(
        Cartesian3,
        Check,
        defaultValue,
        defined,
        defineProperties,
        DeveloperError,
        freezeObject,
        CesiumMath) {
    'use strict';

    /**
     * A 3x3 matrix, indexable as a column-major order array.
     * Constructor parameters are in row-major order for code readability.
     * @alias Matrix3
     * @constructor
     *
     * @param {Number} [column0Row0=0.0] The value for column 0, row 0.
     * @param {Number} [column1Row0=0.0] The value for column 1, row 0.
     * @param {Number} [column2Row0=0.0] The value for column 2, row 0.
     * @param {Number} [column0Row1=0.0] The value for column 0, row 1.
     * @param {Number} [column1Row1=0.0] The value for column 1, row 1.
     * @param {Number} [column2Row1=0.0] The value for column 2, row 1.
     * @param {Number} [column0Row2=0.0] The value for column 0, row 2.
     * @param {Number} [column1Row2=0.0] The value for column 1, row 2.
     * @param {Number} [column2Row2=0.0] The value for column 2, row 2.
     *
     * @see Matrix3.fromColumnMajorArray
     * @see Matrix3.fromRowMajorArray
     * @see Matrix3.fromQuaternion
     * @see Matrix3.fromScale
     * @see Matrix3.fromUniformScale
     * @see Matrix2
     * @see Matrix4
     */
    function Matrix3(column0Row0, column1Row0, column2Row0,
                           column0Row1, column1Row1, column2Row1,
                           column0Row2, column1Row2, column2Row2) {
        this[0] = defaultValue(column0Row0, 0.0);
        this[1] = defaultValue(column0Row1, 0.0);
        this[2] = defaultValue(column0Row2, 0.0);
        this[3] = defaultValue(column1Row0, 0.0);
        this[4] = defaultValue(column1Row1, 0.0);
        this[5] = defaultValue(column1Row2, 0.0);
        this[6] = defaultValue(column2Row0, 0.0);
        this[7] = defaultValue(column2Row1, 0.0);
        this[8] = defaultValue(column2Row2, 0.0);
    }

    /**
     * The number of elements used to pack the object into an array.
     * @type {Number}
     */
    Matrix3.packedLength = 9;

    /**
     * Stores the provided instance into the provided array.
     *
     * @param {Matrix3} value The value to pack.
     * @param {Number[]} array The array to pack into.
     * @param {Number} [startingIndex=0] The index into the array at which to start packing the elements.
     *
     * @returns {Number[]} The array that was packed into
     */
    Matrix3.pack = function(value, array, startingIndex) {
                Check.typeOf.object('value', value);
        Check.defined('array', array);
        
        startingIndex = defaultValue(startingIndex, 0);

        array[startingIndex++] = value[0];
        array[startingIndex++] = value[1];
        array[startingIndex++] = value[2];
        array[startingIndex++] = value[3];
        array[startingIndex++] = value[4];
        array[startingIndex++] = value[5];
        array[startingIndex++] = value[6];
        array[startingIndex++] = value[7];
        array[startingIndex++] = value[8];

        return array;
    };

    /**
     * Retrieves an instance from a packed array.
     *
     * @param {Number[]} array The packed array.
     * @param {Number} [startingIndex=0] The starting index of the element to be unpacked.
     * @param {Matrix3} [result] The object into which to store the result.
     * @returns {Matrix3} The modified result parameter or a new Matrix3 instance if one was not provided.
     */
    Matrix3.unpack = function(array, startingIndex, result) {
                Check.defined('array', array);
        
        startingIndex = defaultValue(startingIndex, 0);

        if (!defined(result)) {
            result = new Matrix3();
        }

        result[0] = array[startingIndex++];
        result[1] = array[startingIndex++];
        result[2] = array[startingIndex++];
        result[3] = array[startingIndex++];
        result[4] = array[startingIndex++];
        result[5] = array[startingIndex++];
        result[6] = array[startingIndex++];
        result[7] = array[startingIndex++];
        result[8] = array[startingIndex++];
        return result;
    };

    /**
     * Duplicates a Matrix3 instance.
     *
     * @param {Matrix3} matrix The matrix to duplicate.
     * @param {Matrix3} [result] The object onto which to store the result.
     * @returns {Matrix3} The modified result parameter or a new Matrix3 instance if one was not provided. (Returns undefined if matrix is undefined)
     */
    Matrix3.clone = function(matrix, result) {
        if (!defined(matrix)) {
            return undefined;
        }
        if (!defined(result)) {
            return new Matrix3(matrix[0], matrix[3], matrix[6],
                               matrix[1], matrix[4], matrix[7],
                               matrix[2], matrix[5], matrix[8]);
        }
        result[0] = matrix[0];
        result[1] = matrix[1];
        result[2] = matrix[2];
        result[3] = matrix[3];
        result[4] = matrix[4];
        result[5] = matrix[5];
        result[6] = matrix[6];
        result[7] = matrix[7];
        result[8] = matrix[8];
        return result;
    };

    /**
     * Creates a Matrix3 from 9 consecutive elements in an array.
     *
     * @param {Number[]} array The array whose 9 consecutive elements correspond to the positions of the matrix.  Assumes column-major order.
     * @param {Number} [startingIndex=0] The offset into the array of the first element, which corresponds to first column first row position in the matrix.
     * @param {Matrix3} [result] The object onto which to store the result.
     * @returns {Matrix3} The modified result parameter or a new Matrix3 instance if one was not provided.
     *
     * @example
     * // Create the Matrix3:
     * // [1.0, 2.0, 3.0]
     * // [1.0, 2.0, 3.0]
     * // [1.0, 2.0, 3.0]
     *
     * var v = [1.0, 1.0, 1.0, 2.0, 2.0, 2.0, 3.0, 3.0, 3.0];
     * var m = Cesium.Matrix3.fromArray(v);
     *
     * // Create same Matrix3 with using an offset into an array
     * var v2 = [0.0, 0.0, 1.0, 1.0, 1.0, 2.0, 2.0, 2.0, 3.0, 3.0, 3.0];
     * var m2 = Cesium.Matrix3.fromArray(v2, 2);
     */
    Matrix3.fromArray = function(array, startingIndex, result) {
                Check.defined('array', array);
        
        startingIndex = defaultValue(startingIndex, 0);

        if (!defined(result)) {
            result = new Matrix3();
        }

        result[0] = array[startingIndex];
        result[1] = array[startingIndex + 1];
        result[2] = array[startingIndex + 2];
        result[3] = array[startingIndex + 3];
        result[4] = array[startingIndex + 4];
        result[5] = array[startingIndex + 5];
        result[6] = array[startingIndex + 6];
        result[7] = array[startingIndex + 7];
        result[8] = array[startingIndex + 8];
        return result;
    };

    /**
     * Creates a Matrix3 instance from a column-major order array.
     *
     * @param {Number[]} values The column-major order array.
     * @param {Matrix3} [result] The object in which the result will be stored, if undefined a new instance will be created.
     * @returns {Matrix3} The modified result parameter, or a new Matrix3 instance if one was not provided.
     */
    Matrix3.fromColumnMajorArray = function(values, result) {
                Check.defined('values', values);
        
        return Matrix3.clone(values, result);
    };

    /**
     * Creates a Matrix3 instance from a row-major order array.
     * The resulting matrix will be in column-major order.
     *
     * @param {Number[]} values The row-major order array.
     * @param {Matrix3} [result] The object in which the result will be stored, if undefined a new instance will be created.
     * @returns {Matrix3} The modified result parameter, or a new Matrix3 instance if one was not provided.
     */
    Matrix3.fromRowMajorArray = function(values, result) {
                Check.defined('values', values);
        
        if (!defined(result)) {
            return new Matrix3(values[0], values[1], values[2],
                               values[3], values[4], values[5],
                               values[6], values[7], values[8]);
        }
        result[0] = values[0];
        result[1] = values[3];
        result[2] = values[6];
        result[3] = values[1];
        result[4] = values[4];
        result[5] = values[7];
        result[6] = values[2];
        result[7] = values[5];
        result[8] = values[8];
        return result;
    };

    /**
     * Computes a 3x3 rotation matrix from the provided quaternion.
     *
     * @param {Quaternion} quaternion the quaternion to use.
     * @param {Matrix3} [result] The object in which the result will be stored, if undefined a new instance will be created.
     * @returns {Matrix3} The 3x3 rotation matrix from this quaternion.
     */
    Matrix3.fromQuaternion = function(quaternion, result) {
                Check.typeOf.object('quaternion', quaternion);
        
        var x2 = quaternion.x * quaternion.x;
        var xy = quaternion.x * quaternion.y;
        var xz = quaternion.x * quaternion.z;
        var xw = quaternion.x * quaternion.w;
        var y2 = quaternion.y * quaternion.y;
        var yz = quaternion.y * quaternion.z;
        var yw = quaternion.y * quaternion.w;
        var z2 = quaternion.z * quaternion.z;
        var zw = quaternion.z * quaternion.w;
        var w2 = quaternion.w * quaternion.w;

        var m00 = x2 - y2 - z2 + w2;
        var m01 = 2.0 * (xy - zw);
        var m02 = 2.0 * (xz + yw);

        var m10 = 2.0 * (xy + zw);
        var m11 = -x2 + y2 - z2 + w2;
        var m12 = 2.0 * (yz - xw);

        var m20 = 2.0 * (xz - yw);
        var m21 = 2.0 * (yz + xw);
        var m22 = -x2 - y2 + z2 + w2;

        if (!defined(result)) {
            return new Matrix3(m00, m01, m02,
                               m10, m11, m12,
                               m20, m21, m22);
        }
        result[0] = m00;
        result[1] = m10;
        result[2] = m20;
        result[3] = m01;
        result[4] = m11;
        result[5] = m21;
        result[6] = m02;
        result[7] = m12;
        result[8] = m22;
        return result;
    };

    /**
     * Computes a 3x3 rotation matrix from the provided headingPitchRoll. (see http://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles )
     *
     * @param {HeadingPitchRoll} headingPitchRoll the headingPitchRoll to use.
     * @param {Matrix3} [result] The object in which the result will be stored, if undefined a new instance will be created.
     * @returns {Matrix3} The 3x3 rotation matrix from this headingPitchRoll.
     */
    Matrix3.fromHeadingPitchRoll = function(headingPitchRoll, result) {
                Check.typeOf.object('headingPitchRoll', headingPitchRoll);
        
        var cosTheta = Math.cos(-headingPitchRoll.pitch);
        var cosPsi = Math.cos(-headingPitchRoll.heading);
        var cosPhi = Math.cos(headingPitchRoll.roll);
        var sinTheta = Math.sin(-headingPitchRoll.pitch);
        var sinPsi = Math.sin(-headingPitchRoll.heading);
        var sinPhi = Math.sin(headingPitchRoll.roll);

        var m00 = cosTheta * cosPsi;
        var m01 = -cosPhi * sinPsi + sinPhi * sinTheta * cosPsi;
        var m02 = sinPhi * sinPsi + cosPhi * sinTheta * cosPsi;

        var m10 = cosTheta * sinPsi;
        var m11 = cosPhi * cosPsi + sinPhi * sinTheta * sinPsi;
        var m12 = -sinPhi * cosPsi + cosPhi * sinTheta * sinPsi;

        var m20 = -sinTheta;
        var m21 = sinPhi * cosTheta;
        var m22 = cosPhi * cosTheta;

        if (!defined(result)) {
            return new Matrix3(m00, m01, m02,
                m10, m11, m12,
                m20, m21, m22);
        }
        result[0] = m00;
        result[1] = m10;
        result[2] = m20;
        result[3] = m01;
        result[4] = m11;
        result[5] = m21;
        result[6] = m02;
        result[7] = m12;
        result[8] = m22;
        return result;
    };

    /**
     * Computes a Matrix3 instance representing a non-uniform scale.
     *
     * @param {Cartesian3} scale The x, y, and z scale factors.
     * @param {Matrix3} [result] The object in which the result will be stored, if undefined a new instance will be created.
     * @returns {Matrix3} The modified result parameter, or a new Matrix3 instance if one was not provided.
     *
     * @example
     * // Creates
     * //   [7.0, 0.0, 0.0]
     * //   [0.0, 8.0, 0.0]
     * //   [0.0, 0.0, 9.0]
     * var m = Cesium.Matrix3.fromScale(new Cesium.Cartesian3(7.0, 8.0, 9.0));
     */
    Matrix3.fromScale = function(scale, result) {
                Check.typeOf.object('scale', scale);
        
        if (!defined(result)) {
            return new Matrix3(
                scale.x, 0.0,     0.0,
                0.0,     scale.y, 0.0,
                0.0,     0.0,     scale.z);
        }

        result[0] = scale.x;
        result[1] = 0.0;
        result[2] = 0.0;
        result[3] = 0.0;
        result[4] = scale.y;
        result[5] = 0.0;
        result[6] = 0.0;
        result[7] = 0.0;
        result[8] = scale.z;
        return result;
    };

    /**
     * Computes a Matrix3 instance representing a uniform scale.
     *
     * @param {Number} scale The uniform scale factor.
     * @param {Matrix3} [result] The object in which the result will be stored, if undefined a new instance will be created.
     * @returns {Matrix3} The modified result parameter, or a new Matrix3 instance if one was not provided.
     *
     * @example
     * // Creates
     * //   [2.0, 0.0, 0.0]
     * //   [0.0, 2.0, 0.0]
     * //   [0.0, 0.0, 2.0]
     * var m = Cesium.Matrix3.fromUniformScale(2.0);
     */
    Matrix3.fromUniformScale = function(scale, result) {
                Check.typeOf.number('scale', scale);
        
        if (!defined(result)) {
            return new Matrix3(
                scale, 0.0,   0.0,
                0.0,   scale, 0.0,
                0.0,   0.0,   scale);
        }

        result[0] = scale;
        result[1] = 0.0;
        result[2] = 0.0;
        result[3] = 0.0;
        result[4] = scale;
        result[5] = 0.0;
        result[6] = 0.0;
        result[7] = 0.0;
        result[8] = scale;
        return result;
    };

    /**
     * Computes a Matrix3 instance representing the cross product equivalent matrix of a Cartesian3 vector.
     *
     * @param {Cartesian3} vector the vector on the left hand side of the cross product operation.
     * @param {Matrix3} [result] The object in which the result will be stored, if undefined a new instance will be created.
     * @returns {Matrix3} The modified result parameter, or a new Matrix3 instance if one was not provided.
     *
     * @example
     * // Creates
     * //   [0.0, -9.0,  8.0]
     * //   [9.0,  0.0, -7.0]
     * //   [-8.0, 7.0,  0.0]
     * var m = Cesium.Matrix3.fromCrossProduct(new Cesium.Cartesian3(7.0, 8.0, 9.0));
     */
    Matrix3.fromCrossProduct = function(vector, result) {
                Check.typeOf.object('vector', vector);
        
        if (!defined(result)) {
            return new Matrix3(
                      0.0, -vector.z,  vector.y,
                 vector.z,       0.0, -vector.x,
                -vector.y,  vector.x,       0.0);
        }

        result[0] = 0.0;
        result[1] = vector.z;
        result[2] = -vector.y;
        result[3] = -vector.z;
        result[4] = 0.0;
        result[5] = vector.x;
        result[6] = vector.y;
        result[7] = -vector.x;
        result[8] = 0.0;
        return result;
    };

    /**
     * Creates a rotation matrix around the x-axis.
     *
     * @param {Number} angle The angle, in radians, of the rotation.  Positive angles are counterclockwise.
     * @param {Matrix3} [result] The object in which the result will be stored, if undefined a new instance will be created.
     * @returns {Matrix3} The modified result parameter, or a new Matrix3 instance if one was not provided.
     *
     * @example
     * // Rotate a point 45 degrees counterclockwise around the x-axis.
     * var p = new Cesium.Cartesian3(5, 6, 7);
     * var m = Cesium.Matrix3.fromRotationX(Cesium.Math.toRadians(45.0));
     * var rotated = Cesium.Matrix3.multiplyByVector(m, p, new Cesium.Cartesian3());
     */
    Matrix3.fromRotationX = function(angle, result) {
                Check.typeOf.number('angle', angle);
        
        var cosAngle = Math.cos(angle);
        var sinAngle = Math.sin(angle);

        if (!defined(result)) {
            return new Matrix3(
                1.0, 0.0, 0.0,
                0.0, cosAngle, -sinAngle,
                0.0, sinAngle, cosAngle);
        }

        result[0] = 1.0;
        result[1] = 0.0;
        result[2] = 0.0;
        result[3] = 0.0;
        result[4] = cosAngle;
        result[5] = sinAngle;
        result[6] = 0.0;
        result[7] = -sinAngle;
        result[8] = cosAngle;

        return result;
    };

    /**
     * Creates a rotation matrix around the y-axis.
     *
     * @param {Number} angle The angle, in radians, of the rotation.  Positive angles are counterclockwise.
     * @param {Matrix3} [result] The object in which the result will be stored, if undefined a new instance will be created.
     * @returns {Matrix3} The modified result parameter, or a new Matrix3 instance if one was not provided.
     *
     * @example
     * // Rotate a point 45 degrees counterclockwise around the y-axis.
     * var p = new Cesium.Cartesian3(5, 6, 7);
     * var m = Cesium.Matrix3.fromRotationY(Cesium.Math.toRadians(45.0));
     * var rotated = Cesium.Matrix3.multiplyByVector(m, p, new Cesium.Cartesian3());
     */
    Matrix3.fromRotationY = function(angle, result) {
                Check.typeOf.number('angle', angle);
        
        var cosAngle = Math.cos(angle);
        var sinAngle = Math.sin(angle);

        if (!defined(result)) {
            return new Matrix3(
                cosAngle, 0.0, sinAngle,
                0.0, 1.0, 0.0,
                -sinAngle, 0.0, cosAngle);
        }

        result[0] = cosAngle;
        result[1] = 0.0;
        result[2] = -sinAngle;
        result[3] = 0.0;
        result[4] = 1.0;
        result[5] = 0.0;
        result[6] = sinAngle;
        result[7] = 0.0;
        result[8] = cosAngle;

        return result;
    };

    /**
     * Creates a rotation matrix around the z-axis.
     *
     * @param {Number} angle The angle, in radians, of the rotation.  Positive angles are counterclockwise.
     * @param {Matrix3} [result] The object in which the result will be stored, if undefined a new instance will be created.
     * @returns {Matrix3} The modified result parameter, or a new Matrix3 instance if one was not provided.
     *
     * @example
     * // Rotate a point 45 degrees counterclockwise around the z-axis.
     * var p = new Cesium.Cartesian3(5, 6, 7);
     * var m = Cesium.Matrix3.fromRotationZ(Cesium.Math.toRadians(45.0));
     * var rotated = Cesium.Matrix3.multiplyByVector(m, p, new Cesium.Cartesian3());
     */
    Matrix3.fromRotationZ = function(angle, result) {
                Check.typeOf.number('angle', angle);
        
        var cosAngle = Math.cos(angle);
        var sinAngle = Math.sin(angle);

        if (!defined(result)) {
            return new Matrix3(
                cosAngle, -sinAngle, 0.0,
                sinAngle, cosAngle, 0.0,
                0.0, 0.0, 1.0);
        }

        result[0] = cosAngle;
        result[1] = sinAngle;
        result[2] = 0.0;
        result[3] = -sinAngle;
        result[4] = cosAngle;
        result[5] = 0.0;
        result[6] = 0.0;
        result[7] = 0.0;
        result[8] = 1.0;

        return result;
    };

    /**
     * Creates an Array from the provided Matrix3 instance.
     * The array will be in column-major order.
     *
     * @param {Matrix3} matrix The matrix to use..
     * @param {Number[]} [result] The Array onto which to store the result.
     * @returns {Number[]} The modified Array parameter or a new Array instance if one was not provided.
     */
    Matrix3.toArray = function(matrix, result) {
                Check.typeOf.object('matrix', matrix);
        
        if (!defined(result)) {
            return [matrix[0], matrix[1], matrix[2], matrix[3], matrix[4], matrix[5], matrix[6], matrix[7], matrix[8]];
        }
        result[0] = matrix[0];
        result[1] = matrix[1];
        result[2] = matrix[2];
        result[3] = matrix[3];
        result[4] = matrix[4];
        result[5] = matrix[5];
        result[6] = matrix[6];
        result[7] = matrix[7];
        result[8] = matrix[8];
        return result;
    };

    /**
     * Computes the array index of the element at the provided row and column.
     *
     * @param {Number} row The zero-based index of the row.
     * @param {Number} column The zero-based index of the column.
     * @returns {Number} The index of the element at the provided row and column.
     *
     * @exception {DeveloperError} row must be 0, 1, or 2.
     * @exception {DeveloperError} column must be 0, 1, or 2.
     *
     * @example
     * var myMatrix = new Cesium.Matrix3();
     * var column1Row0Index = Cesium.Matrix3.getElementIndex(1, 0);
     * var column1Row0 = myMatrix[column1Row0Index]
     * myMatrix[column1Row0Index] = 10.0;
     */
    Matrix3.getElementIndex = function(column, row) {
                Check.typeOf.number.greaterThanOrEquals('row', row, 0);
        Check.typeOf.number.lessThanOrEquals('row', row, 2);
        Check.typeOf.number.greaterThanOrEquals('column', column, 0);
        Check.typeOf.number.lessThanOrEquals('column', column, 2);
        
        return column * 3 + row;
    };

    /**
     * Retrieves a copy of the matrix column at the provided index as a Cartesian3 instance.
     *
     * @param {Matrix3} matrix The matrix to use.
     * @param {Number} index The zero-based index of the column to retrieve.
     * @param {Cartesian3} result The object onto which to store the result.
     * @returns {Cartesian3} The modified result parameter.
     *
     * @exception {DeveloperError} index must be 0, 1, or 2.
     */
    Matrix3.getColumn = function(matrix, index, result) {
                Check.typeOf.object('matrix', matrix);
        Check.typeOf.number.greaterThanOrEquals('index', index, 0);
        Check.typeOf.number.lessThanOrEquals('index', index, 2);
        Check.typeOf.object('result', result);
        
        var startIndex = index * 3;
        var x = matrix[startIndex];
        var y = matrix[startIndex + 1];
        var z = matrix[startIndex + 2];

        result.x = x;
        result.y = y;
        result.z = z;
        return result;
    };

    /**
     * Computes a new matrix that replaces the specified column in the provided matrix with the provided Cartesian3 instance.
     *
     * @param {Matrix3} matrix The matrix to use.
     * @param {Number} index The zero-based index of the column to set.
     * @param {Cartesian3} cartesian The Cartesian whose values will be assigned to the specified column.
     * @param {Matrix3} result The object onto which to store the result.
     * @returns {Matrix3} The modified result parameter.
     *
     * @exception {DeveloperError} index must be 0, 1, or 2.
     */
    Matrix3.setColumn = function(matrix, index, cartesian, result) {
                Check.typeOf.object('matrix', matrix);
        Check.typeOf.number.greaterThanOrEquals('index', index, 0);
        Check.typeOf.number.lessThanOrEquals('index', index, 2);
        Check.typeOf.object('cartesian', cartesian);
        Check.typeOf.object('result', result);
        
        result = Matrix3.clone(matrix, result);
        var startIndex = index * 3;
        result[startIndex] = cartesian.x;
        result[startIndex + 1] = cartesian.y;
        result[startIndex + 2] = cartesian.z;
        return result;
    };

    /**
     * Retrieves a copy of the matrix row at the provided index as a Cartesian3 instance.
     *
     * @param {Matrix3} matrix The matrix to use.
     * @param {Number} index The zero-based index of the row to retrieve.
     * @param {Cartesian3} result The object onto which to store the result.
     * @returns {Cartesian3} The modified result parameter.
     *
     * @exception {DeveloperError} index must be 0, 1, or 2.
     */
    Matrix3.getRow = function(matrix, index, result) {
                Check.typeOf.object('matrix', matrix);
        Check.typeOf.number.greaterThanOrEquals('index', index, 0);
        Check.typeOf.number.lessThanOrEquals('index', index, 2);
        Check.typeOf.object('result', result);
        
        var x = matrix[index];
        var y = matrix[index + 3];
        var z = matrix[index + 6];

        result.x = x;
        result.y = y;
        result.z = z;
        return result;
    };

    /**
     * Computes a new matrix that replaces the specified row in the provided matrix with the provided Cartesian3 instance.
     *
     * @param {Matrix3} matrix The matrix to use.
     * @param {Number} index The zero-based index of the row to set.
     * @param {Cartesian3} cartesian The Cartesian whose values will be assigned to the specified row.
     * @param {Matrix3} result The object onto which to store the result.
     * @returns {Matrix3} The modified result parameter.
     *
     * @exception {DeveloperError} index must be 0, 1, or 2.
     */
    Matrix3.setRow = function(matrix, index, cartesian, result) {
                Check.typeOf.object('matrix', matrix);
        Check.typeOf.number.greaterThanOrEquals('index', index, 0);
        Check.typeOf.number.lessThanOrEquals('index', index, 2);
        Check.typeOf.object('cartesian', cartesian);
        Check.typeOf.object('result', result);
        
        result = Matrix3.clone(matrix, result);
        result[index] = cartesian.x;
        result[index + 3] = cartesian.y;
        result[index + 6] = cartesian.z;
        return result;
    };

    var scratchColumn = new Cartesian3();

    /**
     * Extracts the non-uniform scale assuming the matrix is an affine transformation.
     *
     * @param {Matrix3} matrix The matrix.
     * @param {Cartesian3} result The object onto which to store the result.
     * @returns {Cartesian3} The modified result parameter.
     */
    Matrix3.getScale = function(matrix, result) {
                Check.typeOf.object('matrix', matrix);
        Check.typeOf.object('result', result);
        
        result.x = Cartesian3.magnitude(Cartesian3.fromElements(matrix[0], matrix[1], matrix[2], scratchColumn));
        result.y = Cartesian3.magnitude(Cartesian3.fromElements(matrix[3], matrix[4], matrix[5], scratchColumn));
        result.z = Cartesian3.magnitude(Cartesian3.fromElements(matrix[6], matrix[7], matrix[8], scratchColumn));
        return result;
    };

    var scratchScale = new Cartesian3();

    /**
     * Computes the maximum scale assuming the matrix is an affine transformation.
     * The maximum scale is the maximum length of the column vectors.
     *
     * @param {Matrix3} matrix The matrix.
     * @returns {Number} The maximum scale.
     */
    Matrix3.getMaximumScale = function(matrix) {
        Matrix3.getScale(matrix, scratchScale);
        return Cartesian3.maximumComponent(scratchScale);
    };

    /**
     * Computes the product of two matrices.
     *
     * @param {Matrix3} left The first matrix.
     * @param {Matrix3} right The second matrix.
     * @param {Matrix3} result The object onto which to store the result.
     * @returns {Matrix3} The modified result parameter.
     */
    Matrix3.multiply = function(left, right, result) {
                Check.typeOf.object('left', left);
        Check.typeOf.object('right', right);
        Check.typeOf.object('result', result);
        
        var column0Row0 = left[0] * right[0] + left[3] * right[1] + left[6] * right[2];
        var column0Row1 = left[1] * right[0] + left[4] * right[1] + left[7] * right[2];
        var column0Row2 = left[2] * right[0] + left[5] * right[1] + left[8] * right[2];

        var column1Row0 = left[0] * right[3] + left[3] * right[4] + left[6] * right[5];
        var column1Row1 = left[1] * right[3] + left[4] * right[4] + left[7] * right[5];
        var column1Row2 = left[2] * right[3] + left[5] * right[4] + left[8] * right[5];

        var column2Row0 = left[0] * right[6] + left[3] * right[7] + left[6] * right[8];
        var column2Row1 = left[1] * right[6] + left[4] * right[7] + left[7] * right[8];
        var column2Row2 = left[2] * right[6] + left[5] * right[7] + left[8] * right[8];

        result[0] = column0Row0;
        result[1] = column0Row1;
        result[2] = column0Row2;
        result[3] = column1Row0;
        result[4] = column1Row1;
        result[5] = column1Row2;
        result[6] = column2Row0;
        result[7] = column2Row1;
        result[8] = column2Row2;
        return result;
    };

    /**
     * Computes the sum of two matrices.
     *
     * @param {Matrix3} left The first matrix.
     * @param {Matrix3} right The second matrix.
     * @param {Matrix3} result The object onto which to store the result.
     * @returns {Matrix3} The modified result parameter.
     */
    Matrix3.add = function(left, right, result) {
                Check.typeOf.object('left', left);
        Check.typeOf.object('right', right);
        Check.typeOf.object('result', result);
        
        result[0] = left[0] + right[0];
        result[1] = left[1] + right[1];
        result[2] = left[2] + right[2];
        result[3] = left[3] + right[3];
        result[4] = left[4] + right[4];
        result[5] = left[5] + right[5];
        result[6] = left[6] + right[6];
        result[7] = left[7] + right[7];
        result[8] = left[8] + right[8];
        return result;
    };

    /**
     * Computes the difference of two matrices.
     *
     * @param {Matrix3} left The first matrix.
     * @param {Matrix3} right The second matrix.
     * @param {Matrix3} result The object onto which to store the result.
     * @returns {Matrix3} The modified result parameter.
     */
    Matrix3.subtract = function(left, right, result) {
                Check.typeOf.object('left', left);
        Check.typeOf.object('right', right);
        Check.typeOf.object('result', result);
        
        result[0] = left[0] - right[0];
        result[1] = left[1] - right[1];
        result[2] = left[2] - right[2];
        result[3] = left[3] - right[3];
        result[4] = left[4] - right[4];
        result[5] = left[5] - right[5];
        result[6] = left[6] - right[6];
        result[7] = left[7] - right[7];
        result[8] = left[8] - right[8];
        return result;
    };

    /**
     * Computes the product of a matrix and a column vector.
     *
     * @param {Matrix3} matrix The matrix.
     * @param {Cartesian3} cartesian The column.
     * @param {Cartesian3} result The object onto which to store the result.
     * @returns {Cartesian3} The modified result parameter.
     */
    Matrix3.multiplyByVector = function(matrix, cartesian, result) {
                Check.typeOf.object('matrix', matrix);
        Check.typeOf.object('cartesian', cartesian);
        Check.typeOf.object('result', result);
        
        var vX = cartesian.x;
        var vY = cartesian.y;
        var vZ = cartesian.z;

        var x = matrix[0] * vX + matrix[3] * vY + matrix[6] * vZ;
        var y = matrix[1] * vX + matrix[4] * vY + matrix[7] * vZ;
        var z = matrix[2] * vX + matrix[5] * vY + matrix[8] * vZ;

        result.x = x;
        result.y = y;
        result.z = z;
        return result;
    };

    /**
     * Computes the product of a matrix and a scalar.
     *
     * @param {Matrix3} matrix The matrix.
     * @param {Number} scalar The number to multiply by.
     * @param {Matrix3} result The object onto which to store the result.
     * @returns {Matrix3} The modified result parameter.
     */
    Matrix3.multiplyByScalar = function(matrix, scalar, result) {
                Check.typeOf.object('matrix', matrix);
        Check.typeOf.number('scalar', scalar);
        Check.typeOf.object('result', result);
        
        result[0] = matrix[0] * scalar;
        result[1] = matrix[1] * scalar;
        result[2] = matrix[2] * scalar;
        result[3] = matrix[3] * scalar;
        result[4] = matrix[4] * scalar;
        result[5] = matrix[5] * scalar;
        result[6] = matrix[6] * scalar;
        result[7] = matrix[7] * scalar;
        result[8] = matrix[8] * scalar;
        return result;
    };

    /**
     * Computes the product of a matrix times a (non-uniform) scale, as if the scale were a scale matrix.
     *
     * @param {Matrix3} matrix The matrix on the left-hand side.
     * @param {Cartesian3} scale The non-uniform scale on the right-hand side.
     * @param {Matrix3} result The object onto which to store the result.
     * @returns {Matrix3} The modified result parameter.
     *
     *
     * @example
     * // Instead of Cesium.Matrix3.multiply(m, Cesium.Matrix3.fromScale(scale), m);
     * Cesium.Matrix3.multiplyByScale(m, scale, m);
     *
     * @see Matrix3.fromScale
     * @see Matrix3.multiplyByUniformScale
     */
    Matrix3.multiplyByScale = function(matrix, scale, result) {
                Check.typeOf.object('matrix', matrix);
        Check.typeOf.object('scale', scale);
        Check.typeOf.object('result', result);
        
        result[0] = matrix[0] * scale.x;
        result[1] = matrix[1] * scale.x;
        result[2] = matrix[2] * scale.x;
        result[3] = matrix[3] * scale.y;
        result[4] = matrix[4] * scale.y;
        result[5] = matrix[5] * scale.y;
        result[6] = matrix[6] * scale.z;
        result[7] = matrix[7] * scale.z;
        result[8] = matrix[8] * scale.z;
        return result;
    };

    /**
     * Creates a negated copy of the provided matrix.
     *
     * @param {Matrix3} matrix The matrix to negate.
     * @param {Matrix3} result The object onto which to store the result.
     * @returns {Matrix3} The modified result parameter.
     */
    Matrix3.negate = function(matrix, result) {
                Check.typeOf.object('matrix', matrix);
        Check.typeOf.object('result', result);
        
        result[0] = -matrix[0];
        result[1] = -matrix[1];
        result[2] = -matrix[2];
        result[3] = -matrix[3];
        result[4] = -matrix[4];
        result[5] = -matrix[5];
        result[6] = -matrix[6];
        result[7] = -matrix[7];
        result[8] = -matrix[8];
        return result;
    };

    /**
     * Computes the transpose of the provided matrix.
     *
     * @param {Matrix3} matrix The matrix to transpose.
     * @param {Matrix3} result The object onto which to store the result.
     * @returns {Matrix3} The modified result parameter.
     */
    Matrix3.transpose = function(matrix, result) {
                Check.typeOf.object('matrix', matrix);
        Check.typeOf.object('result', result);
        
        var column0Row0 = matrix[0];
        var column0Row1 = matrix[3];
        var column0Row2 = matrix[6];
        var column1Row0 = matrix[1];
        var column1Row1 = matrix[4];
        var column1Row2 = matrix[7];
        var column2Row0 = matrix[2];
        var column2Row1 = matrix[5];
        var column2Row2 = matrix[8];

        result[0] = column0Row0;
        result[1] = column0Row1;
        result[2] = column0Row2;
        result[3] = column1Row0;
        result[4] = column1Row1;
        result[5] = column1Row2;
        result[6] = column2Row0;
        result[7] = column2Row1;
        result[8] = column2Row2;
        return result;
    };

    function computeFrobeniusNorm(matrix) {
        var norm = 0.0;
        for (var i = 0; i < 9; ++i) {
            var temp = matrix[i];
            norm += temp * temp;
        }

        return Math.sqrt(norm);
    }

    var rowVal = [1, 0, 0];
    var colVal = [2, 2, 1];

    function offDiagonalFrobeniusNorm(matrix) {
        // Computes the "off-diagonal" Frobenius norm.
        // Assumes matrix is symmetric.

        var norm = 0.0;
        for (var i = 0; i < 3; ++i) {
            var temp = matrix[Matrix3.getElementIndex(colVal[i], rowVal[i])];
            norm += 2.0 * temp * temp;
        }

        return Math.sqrt(norm);
    }

    function shurDecomposition(matrix, result) {
        // This routine was created based upon Matrix Computations, 3rd ed., by Golub and Van Loan,
        // section 8.4.2 The 2by2 Symmetric Schur Decomposition.
        //
        // The routine takes a matrix, which is assumed to be symmetric, and
        // finds the largest off-diagonal term, and then creates
        // a matrix (result) which can be used to help reduce it

        var tolerance = CesiumMath.EPSILON15;

        var maxDiagonal = 0.0;
        var rotAxis = 1;

        // find pivot (rotAxis) based on max diagonal of matrix
        for (var i = 0; i < 3; ++i) {
            var temp = Math.abs(matrix[Matrix3.getElementIndex(colVal[i], rowVal[i])]);
            if (temp > maxDiagonal) {
                rotAxis = i;
                maxDiagonal = temp;
            }
        }

        var c = 1.0;
        var s = 0.0;

        var p = rowVal[rotAxis];
        var q = colVal[rotAxis];

        if (Math.abs(matrix[Matrix3.getElementIndex(q, p)]) > tolerance) {
            var qq = matrix[Matrix3.getElementIndex(q, q)];
            var pp = matrix[Matrix3.getElementIndex(p, p)];
            var qp = matrix[Matrix3.getElementIndex(q, p)];

            var tau = (qq - pp) / 2.0 / qp;
            var t;

            if (tau < 0.0) {
                t = -1.0 / (-tau + Math.sqrt(1.0 + tau * tau));
            } else {
                t = 1.0 / (tau + Math.sqrt(1.0 + tau * tau));
            }

            c = 1.0 / Math.sqrt(1.0 + t * t);
            s = t * c;
        }

        result = Matrix3.clone(Matrix3.IDENTITY, result);

        result[Matrix3.getElementIndex(p, p)] = result[Matrix3.getElementIndex(q, q)] = c;
        result[Matrix3.getElementIndex(q, p)] = s;
        result[Matrix3.getElementIndex(p, q)] = -s;

        return result;
    }

    var jMatrix = new Matrix3();
    var jMatrixTranspose = new Matrix3();

    /**
     * Computes the eigenvectors and eigenvalues of a symmetric matrix.
     * <p>
     * Returns a diagonal matrix and unitary matrix such that:
     * <code>matrix = unitary matrix * diagonal matrix * transpose(unitary matrix)</code>
     * </p>
     * <p>
     * The values along the diagonal of the diagonal matrix are the eigenvalues. The columns
     * of the unitary matrix are the corresponding eigenvectors.
     * </p>
     *
     * @param {Matrix3} matrix The matrix to decompose into diagonal and unitary matrix. Expected to be symmetric.
     * @param {Object} [result] An object with unitary and diagonal properties which are matrices onto which to store the result.
     * @returns {Object} An object with unitary and diagonal properties which are the unitary and diagonal matrices, respectively.
     *
     * @example
     * var a = //... symetric matrix
     * var result = {
     *     unitary : new Cesium.Matrix3(),
     *     diagonal : new Cesium.Matrix3()
     * };
     * Cesium.Matrix3.computeEigenDecomposition(a, result);
     *
     * var unitaryTranspose = Cesium.Matrix3.transpose(result.unitary, new Cesium.Matrix3());
     * var b = Cesium.Matrix3.multiply(result.unitary, result.diagonal, new Cesium.Matrix3());
     * Cesium.Matrix3.multiply(b, unitaryTranspose, b); // b is now equal to a
     *
     * var lambda = Cesium.Matrix3.getColumn(result.diagonal, 0, new Cesium.Cartesian3()).x;  // first eigenvalue
     * var v = Cesium.Matrix3.getColumn(result.unitary, 0, new Cesium.Cartesian3());          // first eigenvector
     * var c = Cesium.Cartesian3.multiplyByScalar(v, lambda, new Cesium.Cartesian3());        // equal to Cesium.Matrix3.multiplyByVector(a, v)
     */
    Matrix3.computeEigenDecomposition = function(matrix, result) {
                Check.typeOf.object('matrix', matrix);
        
        // This routine was created based upon Matrix Computations, 3rd ed., by Golub and Van Loan,
        // section 8.4.3 The Classical Jacobi Algorithm

        var tolerance = CesiumMath.EPSILON20;
        var maxSweeps = 10;

        var count = 0;
        var sweep = 0;

        if (!defined(result)) {
            result = {};
        }

        var unitaryMatrix = result.unitary = Matrix3.clone(Matrix3.IDENTITY, result.unitary);
        var diagMatrix = result.diagonal = Matrix3.clone(matrix, result.diagonal);

        var epsilon = tolerance * computeFrobeniusNorm(diagMatrix);

        while (sweep < maxSweeps && offDiagonalFrobeniusNorm(diagMatrix) > epsilon) {
            shurDecomposition(diagMatrix, jMatrix);
            Matrix3.transpose(jMatrix, jMatrixTranspose);
            Matrix3.multiply(diagMatrix, jMatrix, diagMatrix);
            Matrix3.multiply(jMatrixTranspose, diagMatrix, diagMatrix);
            Matrix3.multiply(unitaryMatrix, jMatrix, unitaryMatrix);

            if (++count > 2) {
                ++sweep;
                count = 0;
            }
        }

        return result;
    };

    /**
     * Computes a matrix, which contains the absolute (unsigned) values of the provided matrix's elements.
     *
     * @param {Matrix3} matrix The matrix with signed elements.
     * @param {Matrix3} result The object onto which to store the result.
     * @returns {Matrix3} The modified result parameter.
     */
    Matrix3.abs = function(matrix, result) {
                Check.typeOf.object('matrix', matrix);
        Check.typeOf.object('result', result);
        
        result[0] = Math.abs(matrix[0]);
        result[1] = Math.abs(matrix[1]);
        result[2] = Math.abs(matrix[2]);
        result[3] = Math.abs(matrix[3]);
        result[4] = Math.abs(matrix[4]);
        result[5] = Math.abs(matrix[5]);
        result[6] = Math.abs(matrix[6]);
        result[7] = Math.abs(matrix[7]);
        result[8] = Math.abs(matrix[8]);

        return result;
    };

    /**
     * Computes the determinant of the provided matrix.
     *
     * @param {Matrix3} matrix The matrix to use.
     * @returns {Number} The value of the determinant of the matrix.
     */
    Matrix3.determinant = function(matrix) {
                Check.typeOf.object('matrix', matrix);
        
        var m11 = matrix[0];
        var m21 = matrix[3];
        var m31 = matrix[6];
        var m12 = matrix[1];
        var m22 = matrix[4];
        var m32 = matrix[7];
        var m13 = matrix[2];
        var m23 = matrix[5];
        var m33 = matrix[8];

        return m11 * (m22 * m33 - m23 * m32) + m12 * (m23 * m31 - m21 * m33) + m13 * (m21 * m32 - m22 * m31);
    };

    /**
     * Computes the inverse of the provided matrix.
     *
     * @param {Matrix3} matrix The matrix to invert.
     * @param {Matrix3} result The object onto which to store the result.
     * @returns {Matrix3} The modified result parameter.
     *
     * @exception {DeveloperError} matrix is not invertible.
     */
    Matrix3.inverse = function(matrix, result) {
                Check.typeOf.object('matrix', matrix);
        Check.typeOf.object('result', result);
        
        var m11 = matrix[0];
        var m21 = matrix[1];
        var m31 = matrix[2];
        var m12 = matrix[3];
        var m22 = matrix[4];
        var m32 = matrix[5];
        var m13 = matrix[6];
        var m23 = matrix[7];
        var m33 = matrix[8];

        var determinant = Matrix3.determinant(matrix);

                if (Math.abs(determinant) <= CesiumMath.EPSILON15) {
            throw new DeveloperError('matrix is not invertible');
        }
        
        result[0] = m22 * m33 - m23 * m32;
        result[1] = m23 * m31 - m21 * m33;
        result[2] = m21 * m32 - m22 * m31;
        result[3] = m13 * m32 - m12 * m33;
        result[4] = m11 * m33 - m13 * m31;
        result[5] = m12 * m31 - m11 * m32;
        result[6] = m12 * m23 - m13 * m22;
        result[7] = m13 * m21 - m11 * m23;
        result[8] = m11 * m22 - m12 * m21;

       var scale = 1.0 / determinant;
       return Matrix3.multiplyByScalar(result, scale, result);
    };

    /**
     * Compares the provided matrices componentwise and returns
     * <code>true</code> if they are equal, <code>false</code> otherwise.
     *
     * @param {Matrix3} [left] The first matrix.
     * @param {Matrix3} [right] The second matrix.
     * @returns {Boolean} <code>true</code> if left and right are equal, <code>false</code> otherwise.
     */
    Matrix3.equals = function(left, right) {
        return (left === right) ||
               (defined(left) &&
                defined(right) &&
                left[0] === right[0] &&
                left[1] === right[1] &&
                left[2] === right[2] &&
                left[3] === right[3] &&
                left[4] === right[4] &&
                left[5] === right[5] &&
                left[6] === right[6] &&
                left[7] === right[7] &&
                left[8] === right[8]);
    };

    /**
     * Compares the provided matrices componentwise and returns
     * <code>true</code> if they are within the provided epsilon,
     * <code>false</code> otherwise.
     *
     * @param {Matrix3} [left] The first matrix.
     * @param {Matrix3} [right] The second matrix.
     * @param {Number} epsilon The epsilon to use for equality testing.
     * @returns {Boolean} <code>true</code> if left and right are within the provided epsilon, <code>false</code> otherwise.
     */
    Matrix3.equalsEpsilon = function(left, right, epsilon) {
                Check.typeOf.number('epsilon', epsilon);
        
        return (left === right) ||
                (defined(left) &&
                defined(right) &&
                Math.abs(left[0] - right[0]) <= epsilon &&
                Math.abs(left[1] - right[1]) <= epsilon &&
                Math.abs(left[2] - right[2]) <= epsilon &&
                Math.abs(left[3] - right[3]) <= epsilon &&
                Math.abs(left[4] - right[4]) <= epsilon &&
                Math.abs(left[5] - right[5]) <= epsilon &&
                Math.abs(left[6] - right[6]) <= epsilon &&
                Math.abs(left[7] - right[7]) <= epsilon &&
                Math.abs(left[8] - right[8]) <= epsilon);
    };

    /**
     * An immutable Matrix3 instance initialized to the identity matrix.
     *
     * @type {Matrix3}
     * @constant
     */
    Matrix3.IDENTITY = freezeObject(new Matrix3(1.0, 0.0, 0.0,
                                                0.0, 1.0, 0.0,
                                                0.0, 0.0, 1.0));

    /**
     * An immutable Matrix3 instance initialized to the zero matrix.
     *
     * @type {Matrix3}
     * @constant
     */
    Matrix3.ZERO = freezeObject(new Matrix3(0.0, 0.0, 0.0,
                                            0.0, 0.0, 0.0,
                                            0.0, 0.0, 0.0));

    /**
     * The index into Matrix3 for column 0, row 0.
     *
     * @type {Number}
     * @constant
     */
    Matrix3.COLUMN0ROW0 = 0;

    /**
     * The index into Matrix3 for column 0, row 1.
     *
     * @type {Number}
     * @constant
     */
    Matrix3.COLUMN0ROW1 = 1;

    /**
     * The index into Matrix3 for column 0, row 2.
     *
     * @type {Number}
     * @constant
     */
    Matrix3.COLUMN0ROW2 = 2;

    /**
     * The index into Matrix3 for column 1, row 0.
     *
     * @type {Number}
     * @constant
     */
    Matrix3.COLUMN1ROW0 = 3;

    /**
     * The index into Matrix3 for column 1, row 1.
     *
     * @type {Number}
     * @constant
     */
    Matrix3.COLUMN1ROW1 = 4;

    /**
     * The index into Matrix3 for column 1, row 2.
     *
     * @type {Number}
     * @constant
     */
    Matrix3.COLUMN1ROW2 = 5;

    /**
     * The index into Matrix3 for column 2, row 0.
     *
     * @type {Number}
     * @constant
     */
    Matrix3.COLUMN2ROW0 = 6;

    /**
     * The index into Matrix3 for column 2, row 1.
     *
     * @type {Number}
     * @constant
     */
    Matrix3.COLUMN2ROW1 = 7;

    /**
     * The index into Matrix3 for column 2, row 2.
     *
     * @type {Number}
     * @constant
     */
    Matrix3.COLUMN2ROW2 = 8;

    defineProperties(Matrix3.prototype, {
        /**
         * Gets the number of items in the collection.
         * @memberof Matrix3.prototype
         *
         * @type {Number}
         */
        length : {
            get : function() {
                return Matrix3.packedLength;
            }
        }
    });

    /**
     * Duplicates the provided Matrix3 instance.
     *
     * @param {Matrix3} [result] The object onto which to store the result.
     * @returns {Matrix3} The modified result parameter or a new Matrix3 instance if one was not provided.
     */
    Matrix3.prototype.clone = function(result) {
        return Matrix3.clone(this, result);
    };

    /**
     * Compares this matrix to the provided matrix componentwise and returns
     * <code>true</code> if they are equal, <code>false</code> otherwise.
     *
     * @param {Matrix3} [right] The right hand side matrix.
     * @returns {Boolean} <code>true</code> if they are equal, <code>false</code> otherwise.
     */
    Matrix3.prototype.equals = function(right) {
        return Matrix3.equals(this, right);
    };

    /**
     * @private
     */
    Matrix3.equalsArray = function(matrix, array, offset) {
        return matrix[0] === array[offset] &&
               matrix[1] === array[offset + 1] &&
               matrix[2] === array[offset + 2] &&
               matrix[3] === array[offset + 3] &&
               matrix[4] === array[offset + 4] &&
               matrix[5] === array[offset + 5] &&
               matrix[6] === array[offset + 6] &&
               matrix[7] === array[offset + 7] &&
               matrix[8] === array[offset + 8];
    };

    /**
     * Compares this matrix to the provided matrix componentwise and returns
     * <code>true</code> if they are within the provided epsilon,
     * <code>false</code> otherwise.
     *
     * @param {Matrix3} [right] The right hand side matrix.
     * @param {Number} epsilon The epsilon to use for equality testing.
     * @returns {Boolean} <code>true</code> if they are within the provided epsilon, <code>false</code> otherwise.
     */
    Matrix3.prototype.equalsEpsilon = function(right, epsilon) {
        return Matrix3.equalsEpsilon(this, right, epsilon);
    };

    /**
     * Creates a string representing this Matrix with each row being
     * on a separate line and in the format '(column0, column1, column2)'.
     *
     * @returns {String} A string representing the provided Matrix with each row being on a separate line and in the format '(column0, column1, column2)'.
     */
    Matrix3.prototype.toString = function() {
        return '(' + this[0] + ', ' + this[3] + ', ' + this[6] + ')\n' +
               '(' + this[1] + ', ' + this[4] + ', ' + this[7] + ')\n' +
               '(' + this[2] + ', ' + this[5] + ', ' + this[8] + ')';
    };

    return Matrix3;
});

define('Core/Cartesian4',[
        './Check',
        './defaultValue',
        './defined',
        './DeveloperError',
        './freezeObject',
        './Math'
    ], function(
        Check,
        defaultValue,
        defined,
        DeveloperError,
        freezeObject,
        CesiumMath) {
    'use strict';

    /**
     * A 4D Cartesian point.
     * @alias Cartesian4
     * @constructor
     *
     * @param {Number} [x=0.0] The X component.
     * @param {Number} [y=0.0] The Y component.
     * @param {Number} [z=0.0] The Z component.
     * @param {Number} [w=0.0] The W component.
     *
     * @see Cartesian2
     * @see Cartesian3
     * @see Packable
     */
    function Cartesian4(x, y, z, w) {
        /**
         * The X component.
         * @type {Number}
         * @default 0.0
         */
        this.x = defaultValue(x, 0.0);

        /**
         * The Y component.
         * @type {Number}
         * @default 0.0
         */
        this.y = defaultValue(y, 0.0);

        /**
         * The Z component.
         * @type {Number}
         * @default 0.0
         */
        this.z = defaultValue(z, 0.0);

        /**
         * The W component.
         * @type {Number}
         * @default 0.0
         */
        this.w = defaultValue(w, 0.0);
    }

    /**
     * Creates a Cartesian4 instance from x, y, z and w coordinates.
     *
     * @param {Number} x The x coordinate.
     * @param {Number} y The y coordinate.
     * @param {Number} z The z coordinate.
     * @param {Number} w The w coordinate.
     * @param {Cartesian4} [result] The object onto which to store the result.
     * @returns {Cartesian4} The modified result parameter or a new Cartesian4 instance if one was not provided.
     */
    Cartesian4.fromElements = function(x, y, z, w, result) {
        if (!defined(result)) {
            return new Cartesian4(x, y, z, w);
        }

        result.x = x;
        result.y = y;
        result.z = z;
        result.w = w;
        return result;
    };

    /**
     * Creates a Cartesian4 instance from a {@link Color}. <code>red</code>, <code>green</code>, <code>blue</code>,
     * and <code>alpha</code> map to <code>x</code>, <code>y</code>, <code>z</code>, and <code>w</code>, respectively.
     *
     * @param {Color} color The source color.
     * @param {Cartesian4} [result] The object onto which to store the result.
     * @returns {Cartesian4} The modified result parameter or a new Cartesian4 instance if one was not provided.
     */
    Cartesian4.fromColor = function(color, result) {
                Check.typeOf.object('color', color);
                if (!defined(result)) {
            return new Cartesian4(color.red, color.green, color.blue, color.alpha);
        }

        result.x = color.red;
        result.y = color.green;
        result.z = color.blue;
        result.w = color.alpha;
        return result;
    };

    /**
     * Duplicates a Cartesian4 instance.
     *
     * @param {Cartesian4} cartesian The Cartesian to duplicate.
     * @param {Cartesian4} [result] The object onto which to store the result.
     * @returns {Cartesian4} The modified result parameter or a new Cartesian4 instance if one was not provided. (Returns undefined if cartesian is undefined)
     */
    Cartesian4.clone = function(cartesian, result) {
        if (!defined(cartesian)) {
            return undefined;
        }

        if (!defined(result)) {
            return new Cartesian4(cartesian.x, cartesian.y, cartesian.z, cartesian.w);
        }

        result.x = cartesian.x;
        result.y = cartesian.y;
        result.z = cartesian.z;
        result.w = cartesian.w;
        return result;
    };

    /**
     * The number of elements used to pack the object into an array.
     * @type {Number}
     */
    Cartesian4.packedLength = 4;

    /**
     * Stores the provided instance into the provided array.
     *
     * @param {Cartesian4} value The value to pack.
     * @param {Number[]} array The array to pack into.
     * @param {Number} [startingIndex=0] The index into the array at which to start packing the elements.
     *
     * @returns {Number[]} The array that was packed into
     */
    Cartesian4.pack = function(value, array, startingIndex) {
                Check.typeOf.object('value', value);
        Check.defined('array', array);
        
        startingIndex = defaultValue(startingIndex, 0);

        array[startingIndex++] = value.x;
        array[startingIndex++] = value.y;
        array[startingIndex++] = value.z;
        array[startingIndex] = value.w;

        return array;
    };

    /**
     * Retrieves an instance from a packed array.
     *
     * @param {Number[]} array The packed array.
     * @param {Number} [startingIndex=0] The starting index of the element to be unpacked.
     * @param {Cartesian4} [result] The object into which to store the result.
     * @returns {Cartesian4}  The modified result parameter or a new Cartesian4 instance if one was not provided.
     */
    Cartesian4.unpack = function(array, startingIndex, result) {
                Check.defined('array', array);
        
        startingIndex = defaultValue(startingIndex, 0);

        if (!defined(result)) {
            result = new Cartesian4();
        }
        result.x = array[startingIndex++];
        result.y = array[startingIndex++];
        result.z = array[startingIndex++];
        result.w = array[startingIndex];
        return result;
    };

    /**
     * Flattens an array of Cartesian4s into and array of components.
     *
     * @param {Cartesian4[]} array The array of cartesians to pack.
     * @param {Number[]} [result] The array onto which to store the result.
     * @returns {Number[]} The packed array.
     */
    Cartesian4.packArray = function(array, result) {
                Check.defined('array', array);
        
        var length = array.length;
        if (!defined(result)) {
            result = new Array(length * 4);
        } else {
            result.length = length * 4;
        }

        for (var i = 0; i < length; ++i) {
            Cartesian4.pack(array[i], result, i * 4);
        }
        return result;
    };

    /**
     * Unpacks an array of cartesian components into and array of Cartesian4s.
     *
     * @param {Number[]} array The array of components to unpack.
     * @param {Cartesian4[]} [result] The array onto which to store the result.
     * @returns {Cartesian4[]} The unpacked array.
     */
    Cartesian4.unpackArray = function(array, result) {
                Check.defined('array', array);
        
        var length = array.length;
        if (!defined(result)) {
            result = new Array(length / 4);
        } else {
            result.length = length / 4;
        }

        for (var i = 0; i < length; i += 4) {
            var index = i / 4;
            result[index] = Cartesian4.unpack(array, i, result[index]);
        }
        return result;
    };

    /**
     * Creates a Cartesian4 from four consecutive elements in an array.
     * @function
     *
     * @param {Number[]} array The array whose four consecutive elements correspond to the x, y, z, and w components, respectively.
     * @param {Number} [startingIndex=0] The offset into the array of the first element, which corresponds to the x component.
     * @param {Cartesian4} [result] The object onto which to store the result.
     * @returns {Cartesian4}  The modified result parameter or a new Cartesian4 instance if one was not provided.
     *
     * @example
     * // Create a Cartesian4 with (1.0, 2.0, 3.0, 4.0)
     * var v = [1.0, 2.0, 3.0, 4.0];
     * var p = Cesium.Cartesian4.fromArray(v);
     *
     * // Create a Cartesian4 with (1.0, 2.0, 3.0, 4.0) using an offset into an array
     * var v2 = [0.0, 0.0, 1.0, 2.0, 3.0, 4.0];
     * var p2 = Cesium.Cartesian4.fromArray(v2, 2);
     */
    Cartesian4.fromArray = Cartesian4.unpack;

    /**
     * Computes the value of the maximum component for the supplied Cartesian.
     *
     * @param {Cartesian4} cartesian The cartesian to use.
     * @returns {Number} The value of the maximum component.
     */
    Cartesian4.maximumComponent = function(cartesian) {
                Check.typeOf.object('cartesian', cartesian);
        
        return Math.max(cartesian.x, cartesian.y, cartesian.z, cartesian.w);
    };

    /**
     * Computes the value of the minimum component for the supplied Cartesian.
     *
     * @param {Cartesian4} cartesian The cartesian to use.
     * @returns {Number} The value of the minimum component.
     */
    Cartesian4.minimumComponent = function(cartesian) {
                Check.typeOf.object('cartesian', cartesian);
        
        return Math.min(cartesian.x, cartesian.y, cartesian.z, cartesian.w);
    };

    /**
     * Compares two Cartesians and computes a Cartesian which contains the minimum components of the supplied Cartesians.
     *
     * @param {Cartesian4} first A cartesian to compare.
     * @param {Cartesian4} second A cartesian to compare.
     * @param {Cartesian4} result The object into which to store the result.
     * @returns {Cartesian4} A cartesian with the minimum components.
     */
    Cartesian4.minimumByComponent = function(first, second, result) {
                Check.typeOf.object('first', first);
        Check.typeOf.object('second', second);
        Check.typeOf.object('result', result);
        
        result.x = Math.min(first.x, second.x);
        result.y = Math.min(first.y, second.y);
        result.z = Math.min(first.z, second.z);
        result.w = Math.min(first.w, second.w);

        return result;
    };

    /**
     * Compares two Cartesians and computes a Cartesian which contains the maximum components of the supplied Cartesians.
     *
     * @param {Cartesian4} first A cartesian to compare.
     * @param {Cartesian4} second A cartesian to compare.
     * @param {Cartesian4} result The object into which to store the result.
     * @returns {Cartesian4} A cartesian with the maximum components.
     */
    Cartesian4.maximumByComponent = function(first, second, result) {
                Check.typeOf.object('first', first);
        Check.typeOf.object('second', second);
        Check.typeOf.object('result', result);
        
        result.x = Math.max(first.x, second.x);
        result.y = Math.max(first.y, second.y);
        result.z = Math.max(first.z, second.z);
        result.w = Math.max(first.w, second.w);

        return result;
    };

    /**
     * Computes the provided Cartesian's squared magnitude.
     *
     * @param {Cartesian4} cartesian The Cartesian instance whose squared magnitude is to be computed.
     * @returns {Number} The squared magnitude.
     */
    Cartesian4.magnitudeSquared = function(cartesian) {
                Check.typeOf.object('cartesian', cartesian);
        
        return cartesian.x * cartesian.x + cartesian.y * cartesian.y + cartesian.z * cartesian.z + cartesian.w * cartesian.w;
    };

    /**
     * Computes the Cartesian's magnitude (length).
     *
     * @param {Cartesian4} cartesian The Cartesian instance whose magnitude is to be computed.
     * @returns {Number} The magnitude.
     */
    Cartesian4.magnitude = function(cartesian) {
        return Math.sqrt(Cartesian4.magnitudeSquared(cartesian));
    };

    var distanceScratch = new Cartesian4();

    /**
     * Computes the 4-space distance between two points.
     *
     * @param {Cartesian4} left The first point to compute the distance from.
     * @param {Cartesian4} right The second point to compute the distance to.
     * @returns {Number} The distance between two points.
     *
     * @example
     * // Returns 1.0
     * var d = Cesium.Cartesian4.distance(
     *   new Cesium.Cartesian4(1.0, 0.0, 0.0, 0.0),
     *   new Cesium.Cartesian4(2.0, 0.0, 0.0, 0.0));
     */
    Cartesian4.distance = function(left, right) {
                Check.typeOf.object('left', left);
        Check.typeOf.object('right', right);
        
        Cartesian4.subtract(left, right, distanceScratch);
        return Cartesian4.magnitude(distanceScratch);
    };

    /**
     * Computes the squared distance between two points.  Comparing squared distances
     * using this function is more efficient than comparing distances using {@link Cartesian4#distance}.
     *
     * @param {Cartesian4} left The first point to compute the distance from.
     * @param {Cartesian4} right The second point to compute the distance to.
     * @returns {Number} The distance between two points.
     *
     * @example
     * // Returns 4.0, not 2.0
     * var d = Cesium.Cartesian4.distance(
     *   new Cesium.Cartesian4(1.0, 0.0, 0.0, 0.0),
     *   new Cesium.Cartesian4(3.0, 0.0, 0.0, 0.0));
     */
    Cartesian4.distanceSquared = function(left, right) {
                Check.typeOf.object('left', left);
        Check.typeOf.object('right', right);
        
        Cartesian4.subtract(left, right, distanceScratch);
        return Cartesian4.magnitudeSquared(distanceScratch);
    };

    /**
     * Computes the normalized form of the supplied Cartesian.
     *
     * @param {Cartesian4} cartesian The Cartesian to be normalized.
     * @param {Cartesian4} result The object onto which to store the result.
     * @returns {Cartesian4} The modified result parameter.
     */
    Cartesian4.normalize = function(cartesian, result) {
                Check.typeOf.object('cartesian', cartesian);
        Check.typeOf.object('result', result);
        
        var magnitude = Cartesian4.magnitude(cartesian);

        result.x = cartesian.x / magnitude;
        result.y = cartesian.y / magnitude;
        result.z = cartesian.z / magnitude;
        result.w = cartesian.w / magnitude;

                if (isNaN(result.x) || isNaN(result.y) || isNaN(result.z) || isNaN(result.w)) {
            throw new DeveloperError('normalized result is not a number');
        }
        
        return result;
    };

    /**
     * Computes the dot (scalar) product of two Cartesians.
     *
     * @param {Cartesian4} left The first Cartesian.
     * @param {Cartesian4} right The second Cartesian.
     * @returns {Number} The dot product.
     */
    Cartesian4.dot = function(left, right) {
                Check.typeOf.object('left', left);
        Check.typeOf.object('right', right);
        
        return left.x * right.x + left.y * right.y + left.z * right.z + left.w * right.w;
    };

    /**
     * Computes the componentwise product of two Cartesians.
     *
     * @param {Cartesian4} left The first Cartesian.
     * @param {Cartesian4} right The second Cartesian.
     * @param {Cartesian4} result The object onto which to store the result.
     * @returns {Cartesian4} The modified result parameter.
     */
    Cartesian4.multiplyComponents = function(left, right, result) {
                Check.typeOf.object('left', left);
        Check.typeOf.object('right', right);
        Check.typeOf.object('result', result);
        
        result.x = left.x * right.x;
        result.y = left.y * right.y;
        result.z = left.z * right.z;
        result.w = left.w * right.w;
        return result;
    };

    /**
     * Computes the componentwise quotient of two Cartesians.
     *
     * @param {Cartesian4} left The first Cartesian.
     * @param {Cartesian4} right The second Cartesian.
     * @param {Cartesian4} result The object onto which to store the result.
     * @returns {Cartesian4} The modified result parameter.
     */
    Cartesian4.divideComponents = function(left, right, result) {
                Check.typeOf.object('left', left);
        Check.typeOf.object('right', right);
        Check.typeOf.object('result', result);
        
        result.x = left.x / right.x;
        result.y = left.y / right.y;
        result.z = left.z / right.z;
        result.w = left.w / right.w;
        return result;
    };

    /**
     * Computes the componentwise sum of two Cartesians.
     *
     * @param {Cartesian4} left The first Cartesian.
     * @param {Cartesian4} right The second Cartesian.
     * @param {Cartesian4} result The object onto which to store the result.
     * @returns {Cartesian4} The modified result parameter.
     */
    Cartesian4.add = function(left, right, result) {
                Check.typeOf.object('left', left);
        Check.typeOf.object('right', right);
        Check.typeOf.object('result', result);
        
        result.x = left.x + right.x;
        result.y = left.y + right.y;
        result.z = left.z + right.z;
        result.w = left.w + right.w;
        return result;
    };

    /**
     * Computes the componentwise difference of two Cartesians.
     *
     * @param {Cartesian4} left The first Cartesian.
     * @param {Cartesian4} right The second Cartesian.
     * @param {Cartesian4} result The object onto which to store the result.
     * @returns {Cartesian4} The modified result parameter.
     */
    Cartesian4.subtract = function(left, right, result) {
                Check.typeOf.object('left', left);
        Check.typeOf.object('right', right);
        Check.typeOf.object('result', result);
        
        result.x = left.x - right.x;
        result.y = left.y - right.y;
        result.z = left.z - right.z;
        result.w = left.w - right.w;
        return result;
    };

    /**
     * Multiplies the provided Cartesian componentwise by the provided scalar.
     *
     * @param {Cartesian4} cartesian The Cartesian to be scaled.
     * @param {Number} scalar The scalar to multiply with.
     * @param {Cartesian4} result The object onto which to store the result.
     * @returns {Cartesian4} The modified result parameter.
     */
    Cartesian4.multiplyByScalar = function(cartesian, scalar, result) {
                Check.typeOf.object('cartesian', cartesian);
        Check.typeOf.number('scalar', scalar);
        Check.typeOf.object('result', result);
        
        result.x = cartesian.x * scalar;
        result.y = cartesian.y * scalar;
        result.z = cartesian.z * scalar;
        result.w = cartesian.w * scalar;
        return result;
    };

    /**
     * Divides the provided Cartesian componentwise by the provided scalar.
     *
     * @param {Cartesian4} cartesian The Cartesian to be divided.
     * @param {Number} scalar The scalar to divide by.
     * @param {Cartesian4} result The object onto which to store the result.
     * @returns {Cartesian4} The modified result parameter.
     */
    Cartesian4.divideByScalar = function(cartesian, scalar, result) {
                Check.typeOf.object('cartesian', cartesian);
        Check.typeOf.number('scalar', scalar);
        Check.typeOf.object('result', result);
        
        result.x = cartesian.x / scalar;
        result.y = cartesian.y / scalar;
        result.z = cartesian.z / scalar;
        result.w = cartesian.w / scalar;
        return result;
    };

    /**
     * Negates the provided Cartesian.
     *
     * @param {Cartesian4} cartesian The Cartesian to be negated.
     * @param {Cartesian4} result The object onto which to store the result.
     * @returns {Cartesian4} The modified result parameter.
     */
    Cartesian4.negate = function(cartesian, result) {
                Check.typeOf.object('cartesian', cartesian);
        Check.typeOf.object('result', result);
        
        result.x = -cartesian.x;
        result.y = -cartesian.y;
        result.z = -cartesian.z;
        result.w = -cartesian.w;
        return result;
    };

    /**
     * Computes the absolute value of the provided Cartesian.
     *
     * @param {Cartesian4} cartesian The Cartesian whose absolute value is to be computed.
     * @param {Cartesian4} result The object onto which to store the result.
     * @returns {Cartesian4} The modified result parameter.
     */
    Cartesian4.abs = function(cartesian, result) {
                Check.typeOf.object('cartesian', cartesian);
        Check.typeOf.object('result', result);
        
        result.x = Math.abs(cartesian.x);
        result.y = Math.abs(cartesian.y);
        result.z = Math.abs(cartesian.z);
        result.w = Math.abs(cartesian.w);
        return result;
    };

    var lerpScratch = new Cartesian4();
    /**
     * Computes the linear interpolation or extrapolation at t using the provided cartesians.
     *
     * @param {Cartesian4} start The value corresponding to t at 0.0.
     * @param {Cartesian4}end The value corresponding to t at 1.0.
     * @param {Number} t The point along t at which to interpolate.
     * @param {Cartesian4} result The object onto which to store the result.
     * @returns {Cartesian4} The modified result parameter.
     */
    Cartesian4.lerp = function(start, end, t, result) {
                Check.typeOf.object('start', start);
        Check.typeOf.object('end', end);
        Check.typeOf.number('t', t);
        Check.typeOf.object('result', result);
        
        Cartesian4.multiplyByScalar(end, t, lerpScratch);
        result = Cartesian4.multiplyByScalar(start, 1.0 - t, result);
        return Cartesian4.add(lerpScratch, result, result);
    };

    var mostOrthogonalAxisScratch = new Cartesian4();
    /**
     * Returns the axis that is most orthogonal to the provided Cartesian.
     *
     * @param {Cartesian4} cartesian The Cartesian on which to find the most orthogonal axis.
     * @param {Cartesian4} result The object onto which to store the result.
     * @returns {Cartesian4} The most orthogonal axis.
     */
    Cartesian4.mostOrthogonalAxis = function(cartesian, result) {
                Check.typeOf.object('cartesian', cartesian);
        Check.typeOf.object('result', result);
        
        var f = Cartesian4.normalize(cartesian, mostOrthogonalAxisScratch);
        Cartesian4.abs(f, f);

        if (f.x <= f.y) {
            if (f.x <= f.z) {
                if (f.x <= f.w) {
                    result = Cartesian4.clone(Cartesian4.UNIT_X, result);
                } else {
                    result = Cartesian4.clone(Cartesian4.UNIT_W, result);
                }
            } else if (f.z <= f.w) {
                result = Cartesian4.clone(Cartesian4.UNIT_Z, result);
            } else {
                result = Cartesian4.clone(Cartesian4.UNIT_W, result);
            }
        } else if (f.y <= f.z) {
            if (f.y <= f.w) {
                result = Cartesian4.clone(Cartesian4.UNIT_Y, result);
            } else {
                result = Cartesian4.clone(Cartesian4.UNIT_W, result);
            }
        } else if (f.z <= f.w) {
            result = Cartesian4.clone(Cartesian4.UNIT_Z, result);
        } else {
            result = Cartesian4.clone(Cartesian4.UNIT_W, result);
        }

        return result;
    };

    /**
     * Compares the provided Cartesians componentwise and returns
     * <code>true</code> if they are equal, <code>false</code> otherwise.
     *
     * @param {Cartesian4} [left] The first Cartesian.
     * @param {Cartesian4} [right] The second Cartesian.
     * @returns {Boolean} <code>true</code> if left and right are equal, <code>false</code> otherwise.
     */
    Cartesian4.equals = function(left, right) {
        return (left === right) ||
               ((defined(left)) &&
                (defined(right)) &&
                (left.x === right.x) &&
                (left.y === right.y) &&
                (left.z === right.z) &&
                (left.w === right.w));
    };

    /**
     * @private
     */
    Cartesian4.equalsArray = function(cartesian, array, offset) {
        return cartesian.x === array[offset] &&
               cartesian.y === array[offset + 1] &&
               cartesian.z === array[offset + 2] &&
               cartesian.w === array[offset + 3];
    };

    /**
     * Compares the provided Cartesians componentwise and returns
     * <code>true</code> if they pass an absolute or relative tolerance test,
     * <code>false</code> otherwise.
     *
     * @param {Cartesian4} [left] The first Cartesian.
     * @param {Cartesian4} [right] The second Cartesian.
     * @param {Number} relativeEpsilon The relative epsilon tolerance to use for equality testing.
     * @param {Number} [absoluteEpsilon=relativeEpsilon] The absolute epsilon tolerance to use for equality testing.
     * @returns {Boolean} <code>true</code> if left and right are within the provided epsilon, <code>false</code> otherwise.
     */
    Cartesian4.equalsEpsilon = function(left, right, relativeEpsilon, absoluteEpsilon) {
        return (left === right) ||
               (defined(left) &&
                defined(right) &&
                CesiumMath.equalsEpsilon(left.x, right.x, relativeEpsilon, absoluteEpsilon) &&
                CesiumMath.equalsEpsilon(left.y, right.y, relativeEpsilon, absoluteEpsilon) &&
                CesiumMath.equalsEpsilon(left.z, right.z, relativeEpsilon, absoluteEpsilon) &&
                CesiumMath.equalsEpsilon(left.w, right.w, relativeEpsilon, absoluteEpsilon));
    };

    /**
     * An immutable Cartesian4 instance initialized to (0.0, 0.0, 0.0, 0.0).
     *
     * @type {Cartesian4}
     * @constant
     */
    Cartesian4.ZERO = freezeObject(new Cartesian4(0.0, 0.0, 0.0, 0.0));

    /**
     * An immutable Cartesian4 instance initialized to (1.0, 0.0, 0.0, 0.0).
     *
     * @type {Cartesian4}
     * @constant
     */
    Cartesian4.UNIT_X = freezeObject(new Cartesian4(1.0, 0.0, 0.0, 0.0));

    /**
     * An immutable Cartesian4 instance initialized to (0.0, 1.0, 0.0, 0.0).
     *
     * @type {Cartesian4}
     * @constant
     */
    Cartesian4.UNIT_Y = freezeObject(new Cartesian4(0.0, 1.0, 0.0, 0.0));

    /**
     * An immutable Cartesian4 instance initialized to (0.0, 0.0, 1.0, 0.0).
     *
     * @type {Cartesian4}
     * @constant
     */
    Cartesian4.UNIT_Z = freezeObject(new Cartesian4(0.0, 0.0, 1.0, 0.0));

    /**
     * An immutable Cartesian4 instance initialized to (0.0, 0.0, 0.0, 1.0).
     *
     * @type {Cartesian4}
     * @constant
     */
    Cartesian4.UNIT_W = freezeObject(new Cartesian4(0.0, 0.0, 0.0, 1.0));

    /**
     * Duplicates this Cartesian4 instance.
     *
     * @param {Cartesian4} [result] The object onto which to store the result.
     * @returns {Cartesian4} The modified result parameter or a new Cartesian4 instance if one was not provided.
     */
    Cartesian4.prototype.clone = function(result) {
        return Cartesian4.clone(this, result);
    };

    /**
     * Compares this Cartesian against the provided Cartesian componentwise and returns
     * <code>true</code> if they are equal, <code>false</code> otherwise.
     *
     * @param {Cartesian4} [right] The right hand side Cartesian.
     * @returns {Boolean} <code>true</code> if they are equal, <code>false</code> otherwise.
     */
    Cartesian4.prototype.equals = function(right) {
        return Cartesian4.equals(this, right);
    };

    /**
     * Compares this Cartesian against the provided Cartesian componentwise and returns
     * <code>true</code> if they pass an absolute or relative tolerance test,
     * <code>false</code> otherwise.
     *
     * @param {Cartesian4} [right] The right hand side Cartesian.
     * @param {Number} relativeEpsilon The relative epsilon tolerance to use for equality testing.
     * @param {Number} [absoluteEpsilon=relativeEpsilon] The absolute epsilon tolerance to use for equality testing.
     * @returns {Boolean} <code>true</code> if they are within the provided epsilon, <code>false</code> otherwise.
     */
    Cartesian4.prototype.equalsEpsilon = function(right, relativeEpsilon, absoluteEpsilon) {
        return Cartesian4.equalsEpsilon(this, right, relativeEpsilon, absoluteEpsilon);
    };

    /**
     * Creates a string representing this Cartesian in the format '(x, y)'.
     *
     * @returns {String} A string representing the provided Cartesian in the format '(x, y)'.
     */
    Cartesian4.prototype.toString = function() {
        return '(' + this.x + ', ' + this.y + ', ' + this.z + ', ' + this.w + ')';
    };

    var scratchFloatArray = new Float32Array(1);
    var SHIFT_LEFT_8 = 256.0;
    var SHIFT_LEFT_16 = 65536.0;
    var SHIFT_LEFT_24 = 16777216.0;

    var SHIFT_RIGHT_8 = 1.0 / SHIFT_LEFT_8;
    var SHIFT_RIGHT_16 = 1.0 / SHIFT_LEFT_16;
    var SHIFT_RIGHT_24 = 1.0 / SHIFT_LEFT_24;

    var BIAS = 38.0;

    /**
     * Packs an arbitrary floating point value to 4 values representable using uint8.
     *
     * @param {Number} value A floating point number
     * @param {Cartesian4} [result] The Cartesian4 that will contain the packed float.
     * @returns {Cartesian4} A Cartesian4 representing the float packed to values in x, y, z, and w.
     */
    Cartesian4.packFloat = function(value, result) {
                Check.typeOf.number('value', value);
        
        if (!defined(result)) {
            result = new Cartesian4();
        }

        // Force the value to 32 bit precision
        scratchFloatArray[0] = value;
        value = scratchFloatArray[0];

        if (value === 0.0) {
            return Cartesian4.clone(Cartesian4.ZERO, result);
        }

        var sign = value < 0.0 ? 1.0 : 0.0;
        var exponent;

        if (!isFinite(value)) {
            value = 0.1;
            exponent = BIAS;
        } else {
            value = Math.abs(value);
            exponent = Math.floor(CesiumMath.logBase(value, 10)) + 1.0;
            value = value / Math.pow(10.0, exponent);
        }

        var temp = value * SHIFT_LEFT_8;
        result.x = Math.floor(temp);
        temp = (temp - result.x) * SHIFT_LEFT_8;
        result.y = Math.floor(temp);
        temp = (temp - result.y) * SHIFT_LEFT_8;
        result.z = Math.floor(temp);
        result.w = (exponent + BIAS) * 2.0 + sign;

        return result;
    };

    /**
     * Unpacks a float packed using Cartesian4.packFloat.
     *
     * @param {Cartesian4} packedFloat A Cartesian4 containing a float packed to 4 values representable using uint8.
     * @returns {Number} The unpacked float.
     * @private
     */
    Cartesian4.unpackFloat = function(packedFloat) {
                Check.typeOf.object('packedFloat', packedFloat);
        
        var temp = packedFloat.w / 2.0;
        var exponent = Math.floor(temp);
        var sign = (temp - exponent) * 2.0;
        exponent = exponent - BIAS;

        sign = sign * 2.0 - 1.0;
        sign = -sign;

        if (exponent >= BIAS) {
            return sign < 0.0 ? Number.NEGATIVE_INFINITY : Number.POSITIVE_INFINITY;
        }

        var unpacked = sign * packedFloat.x * SHIFT_RIGHT_8;
        unpacked += sign * packedFloat.y * SHIFT_RIGHT_16;
        unpacked += sign * packedFloat.z * SHIFT_RIGHT_24;

        return unpacked * Math.pow(10.0, exponent);
    };

    return Cartesian4;
});

define('Core/RuntimeError',[
        './defined'
    ], function(
        defined) {
    'use strict';

    /**
     * Constructs an exception object that is thrown due to an error that can occur at runtime, e.g.,
     * out of memory, could not compile shader, etc.  If a function may throw this
     * exception, the calling code should be prepared to catch it.
     * <br /><br />
     * On the other hand, a {@link DeveloperError} indicates an exception due
     * to a developer error, e.g., invalid argument, that usually indicates a bug in the
     * calling code.
     *
     * @alias RuntimeError
     * @constructor
     * @extends Error
     *
     * @param {String} [message] The error message for this exception.
     *
     * @see DeveloperError
     */
    function RuntimeError(message) {
        /**
         * 'RuntimeError' indicating that this exception was thrown due to a runtime error.
         * @type {String}
         * @readonly
         */
        this.name = 'RuntimeError';

        /**
         * The explanation for why this exception was thrown.
         * @type {String}
         * @readonly
         */
        this.message = message;

        //Browsers such as IE don't have a stack property until you actually throw the error.
        var stack;
        try {
            throw new Error();
        } catch (e) {
            stack = e.stack;
        }

        /**
         * The stack trace of this exception, if available.
         * @type {String}
         * @readonly
         */
        this.stack = stack;
    }

    if (defined(Object.create)) {
        RuntimeError.prototype = Object.create(Error.prototype);
        RuntimeError.prototype.constructor = RuntimeError;
    }

    RuntimeError.prototype.toString = function() {
        var str = this.name + ': ' + this.message;

        if (defined(this.stack)) {
            str += '\n' + this.stack.toString();
        }

        return str;
    };

    return RuntimeError;
});

define('Core/Matrix4',[
        './Cartesian3',
        './Cartesian4',
        './Check',
        './defaultValue',
        './defined',
        './defineProperties',
        './freezeObject',
        './Math',
        './Matrix3',
        './RuntimeError'
    ], function(
        Cartesian3,
        Cartesian4,
        Check,
        defaultValue,
        defined,
        defineProperties,
        freezeObject,
        CesiumMath,
        Matrix3,
        RuntimeError) {
    'use strict';

    /**
     * A 4x4 matrix, indexable as a column-major order array.
     * Constructor parameters are in row-major order for code readability.
     * @alias Matrix4
     * @constructor
     *
     * @param {Number} [column0Row0=0.0] The value for column 0, row 0.
     * @param {Number} [column1Row0=0.0] The value for column 1, row 0.
     * @param {Number} [column2Row0=0.0] The value for column 2, row 0.
     * @param {Number} [column3Row0=0.0] The value for column 3, row 0.
     * @param {Number} [column0Row1=0.0] The value for column 0, row 1.
     * @param {Number} [column1Row1=0.0] The value for column 1, row 1.
     * @param {Number} [column2Row1=0.0] The value for column 2, row 1.
     * @param {Number} [column3Row1=0.0] The value for column 3, row 1.
     * @param {Number} [column0Row2=0.0] The value for column 0, row 2.
     * @param {Number} [column1Row2=0.0] The value for column 1, row 2.
     * @param {Number} [column2Row2=0.0] The value for column 2, row 2.
     * @param {Number} [column3Row2=0.0] The value for column 3, row 2.
     * @param {Number} [column0Row3=0.0] The value for column 0, row 3.
     * @param {Number} [column1Row3=0.0] The value for column 1, row 3.
     * @param {Number} [column2Row3=0.0] The value for column 2, row 3.
     * @param {Number} [column3Row3=0.0] The value for column 3, row 3.
     *
     * @see Matrix4.fromColumnMajorArray
     * @see Matrix4.fromRowMajorArray
     * @see Matrix4.fromRotationTranslation
     * @see Matrix4.fromTranslationRotationScale
     * @see Matrix4.fromTranslationQuaternionRotationScale
     * @see Matrix4.fromTranslation
     * @see Matrix4.fromScale
     * @see Matrix4.fromUniformScale
     * @see Matrix4.fromCamera
     * @see Matrix4.computePerspectiveFieldOfView
     * @see Matrix4.computeOrthographicOffCenter
     * @see Matrix4.computePerspectiveOffCenter
     * @see Matrix4.computeInfinitePerspectiveOffCenter
     * @see Matrix4.computeViewportTransformation
     * @see Matrix4.computeView
     * @see Matrix2
     * @see Matrix3
     * @see Packable
     */
    function Matrix4(column0Row0, column1Row0, column2Row0, column3Row0,
                           column0Row1, column1Row1, column2Row1, column3Row1,
                           column0Row2, column1Row2, column2Row2, column3Row2,
                           column0Row3, column1Row3, column2Row3, column3Row3) {
        this[0] = defaultValue(column0Row0, 0.0);
        this[1] = defaultValue(column0Row1, 0.0);
        this[2] = defaultValue(column0Row2, 0.0);
        this[3] = defaultValue(column0Row3, 0.0);
        this[4] = defaultValue(column1Row0, 0.0);
        this[5] = defaultValue(column1Row1, 0.0);
        this[6] = defaultValue(column1Row2, 0.0);
        this[7] = defaultValue(column1Row3, 0.0);
        this[8] = defaultValue(column2Row0, 0.0);
        this[9] = defaultValue(column2Row1, 0.0);
        this[10] = defaultValue(column2Row2, 0.0);
        this[11] = defaultValue(column2Row3, 0.0);
        this[12] = defaultValue(column3Row0, 0.0);
        this[13] = defaultValue(column3Row1, 0.0);
        this[14] = defaultValue(column3Row2, 0.0);
        this[15] = defaultValue(column3Row3, 0.0);
    }

    /**
     * The number of elements used to pack the object into an array.
     * @type {Number}
     */
    Matrix4.packedLength = 16;

    /**
     * Stores the provided instance into the provided array.
     *
     * @param {Matrix4} value The value to pack.
     * @param {Number[]} array The array to pack into.
     * @param {Number} [startingIndex=0] The index into the array at which to start packing the elements.
     *
     * @returns {Number[]} The array that was packed into
     */
    Matrix4.pack = function(value, array, startingIndex) {
                Check.typeOf.object('value', value);
        Check.defined('array', array);
        
        startingIndex = defaultValue(startingIndex, 0);

        array[startingIndex++] = value[0];
        array[startingIndex++] = value[1];
        array[startingIndex++] = value[2];
        array[startingIndex++] = value[3];
        array[startingIndex++] = value[4];
        array[startingIndex++] = value[5];
        array[startingIndex++] = value[6];
        array[startingIndex++] = value[7];
        array[startingIndex++] = value[8];
        array[startingIndex++] = value[9];
        array[startingIndex++] = value[10];
        array[startingIndex++] = value[11];
        array[startingIndex++] = value[12];
        array[startingIndex++] = value[13];
        array[startingIndex++] = value[14];
        array[startingIndex] = value[15];

        return array;
    };

    /**
     * Retrieves an instance from a packed array.
     *
     * @param {Number[]} array The packed array.
     * @param {Number} [startingIndex=0] The starting index of the element to be unpacked.
     * @param {Matrix4} [result] The object into which to store the result.
     * @returns {Matrix4} The modified result parameter or a new Matrix4 instance if one was not provided.
     */
    Matrix4.unpack = function(array, startingIndex, result) {
                Check.defined('array', array);
        
        startingIndex = defaultValue(startingIndex, 0);

        if (!defined(result)) {
            result = new Matrix4();
        }

        result[0] = array[startingIndex++];
        result[1] = array[startingIndex++];
        result[2] = array[startingIndex++];
        result[3] = array[startingIndex++];
        result[4] = array[startingIndex++];
        result[5] = array[startingIndex++];
        result[6] = array[startingIndex++];
        result[7] = array[startingIndex++];
        result[8] = array[startingIndex++];
        result[9] = array[startingIndex++];
        result[10] = array[startingIndex++];
        result[11] = array[startingIndex++];
        result[12] = array[startingIndex++];
        result[13] = array[startingIndex++];
        result[14] = array[startingIndex++];
        result[15] = array[startingIndex];
        return result;
    };

    /**
     * Duplicates a Matrix4 instance.
     *
     * @param {Matrix4} matrix The matrix to duplicate.
     * @param {Matrix4} [result] The object onto which to store the result.
     * @returns {Matrix4} The modified result parameter or a new Matrix4 instance if one was not provided. (Returns undefined if matrix is undefined)
     */
    Matrix4.clone = function(matrix, result) {
        if (!defined(matrix)) {
            return undefined;
        }
        if (!defined(result)) {
            return new Matrix4(matrix[0], matrix[4], matrix[8], matrix[12],
                               matrix[1], matrix[5], matrix[9], matrix[13],
                               matrix[2], matrix[6], matrix[10], matrix[14],
                               matrix[3], matrix[7], matrix[11], matrix[15]);
        }
        result[0] = matrix[0];
        result[1] = matrix[1];
        result[2] = matrix[2];
        result[3] = matrix[3];
        result[4] = matrix[4];
        result[5] = matrix[5];
        result[6] = matrix[6];
        result[7] = matrix[7];
        result[8] = matrix[8];
        result[9] = matrix[9];
        result[10] = matrix[10];
        result[11] = matrix[11];
        result[12] = matrix[12];
        result[13] = matrix[13];
        result[14] = matrix[14];
        result[15] = matrix[15];
        return result;
    };

    /**
     * Creates a Matrix4 from 16 consecutive elements in an array.
     * @function
     *
     * @param {Number[]} array The array whose 16 consecutive elements correspond to the positions of the matrix.  Assumes column-major order.
     * @param {Number} [startingIndex=0] The offset into the array of the first element, which corresponds to first column first row position in the matrix.
     * @param {Matrix4} [result] The object onto which to store the result.
     * @returns {Matrix4} The modified result parameter or a new Matrix4 instance if one was not provided.
     *
     * @example
     * // Create the Matrix4:
     * // [1.0, 2.0, 3.0, 4.0]
     * // [1.0, 2.0, 3.0, 4.0]
     * // [1.0, 2.0, 3.0, 4.0]
     * // [1.0, 2.0, 3.0, 4.0]
     *
     * var v = [1.0, 1.0, 1.0, 1.0, 2.0, 2.0, 2.0, 2.0, 3.0, 3.0, 3.0, 3.0, 4.0, 4.0, 4.0, 4.0];
     * var m = Cesium.Matrix4.fromArray(v);
     *
     * // Create same Matrix4 with using an offset into an array
     * var v2 = [0.0, 0.0, 1.0, 1.0, 1.0, 1.0, 2.0, 2.0, 2.0, 2.0, 3.0, 3.0, 3.0, 3.0, 4.0, 4.0, 4.0, 4.0];
     * var m2 = Cesium.Matrix4.fromArray(v2, 2);
     */
    Matrix4.fromArray = Matrix4.unpack;

    /**
     * Computes a Matrix4 instance from a column-major order array.
     *
     * @param {Number[]} values The column-major order array.
     * @param {Matrix4} [result] The object in which the result will be stored, if undefined a new instance will be created.
     * @returns {Matrix4} The modified result parameter, or a new Matrix4 instance if one was not provided.
     */
    Matrix4.fromColumnMajorArray = function(values, result) {
                Check.defined('values', values);
        
        return Matrix4.clone(values, result);
    };

    /**
     * Computes a Matrix4 instance from a row-major order array.
     * The resulting matrix will be in column-major order.
     *
     * @param {Number[]} values The row-major order array.
     * @param {Matrix4} [result] The object in which the result will be stored, if undefined a new instance will be created.
     * @returns {Matrix4} The modified result parameter, or a new Matrix4 instance if one was not provided.
     */
    Matrix4.fromRowMajorArray = function(values, result) {
                Check.defined('values', values);
        
        if (!defined(result)) {
            return new Matrix4(values[0], values[1], values[2], values[3],
                               values[4], values[5], values[6], values[7],
                               values[8], values[9], values[10], values[11],
                               values[12], values[13], values[14], values[15]);
        }
        result[0] = values[0];
        result[1] = values[4];
        result[2] = values[8];
        result[3] = values[12];
        result[4] = values[1];
        result[5] = values[5];
        result[6] = values[9];
        result[7] = values[13];
        result[8] = values[2];
        result[9] = values[6];
        result[10] = values[10];
        result[11] = values[14];
        result[12] = values[3];
        result[13] = values[7];
        result[14] = values[11];
        result[15] = values[15];
        return result;
    };

    /**
     * Computes a Matrix4 instance from a Matrix3 representing the rotation
     * and a Cartesian3 representing the translation.
     *
     * @param {Matrix3} rotation The upper left portion of the matrix representing the rotation.
     * @param {Cartesian3} [translation=Cartesian3.ZERO] The upper right portion of the matrix representing the translation.
     * @param {Matrix4} [result] The object in which the result will be stored, if undefined a new instance will be created.
     * @returns {Matrix4} The modified result parameter, or a new Matrix4 instance if one was not provided.
     */
    Matrix4.fromRotationTranslation = function(rotation, translation, result) {
                Check.typeOf.object('rotation', rotation);
        
        translation = defaultValue(translation, Cartesian3.ZERO);

        if (!defined(result)) {
            return new Matrix4(rotation[0], rotation[3], rotation[6], translation.x,
                               rotation[1], rotation[4], rotation[7], translation.y,
                               rotation[2], rotation[5], rotation[8], translation.z,
                                       0.0,         0.0,         0.0,           1.0);
        }

        result[0] = rotation[0];
        result[1] = rotation[1];
        result[2] = rotation[2];
        result[3] = 0.0;
        result[4] = rotation[3];
        result[5] = rotation[4];
        result[6] = rotation[5];
        result[7] = 0.0;
        result[8] = rotation[6];
        result[9] = rotation[7];
        result[10] = rotation[8];
        result[11] = 0.0;
        result[12] = translation.x;
        result[13] = translation.y;
        result[14] = translation.z;
        result[15] = 1.0;
        return result;
    };

    /**
     * Computes a Matrix4 instance from a translation, rotation, and scale (TRS)
     * representation with the rotation represented as a quaternion.
     *
     * @param {Cartesian3} translation The translation transformation.
     * @param {Quaternion} rotation The rotation transformation.
     * @param {Cartesian3} scale The non-uniform scale transformation.
     * @param {Matrix4} [result] The object in which the result will be stored, if undefined a new instance will be created.
     * @returns {Matrix4} The modified result parameter, or a new Matrix4 instance if one was not provided.
     *
     * @example
     * var result = Cesium.Matrix4.fromTranslationQuaternionRotationScale(
     *   new Cesium.Cartesian3(1.0, 2.0, 3.0), // translation
     *   Cesium.Quaternion.IDENTITY,           // rotation
     *   new Cesium.Cartesian3(7.0, 8.0, 9.0), // scale
     *   result);
     */
    Matrix4.fromTranslationQuaternionRotationScale = function(translation, rotation, scale, result) {
                Check.typeOf.object('translation', translation);
        Check.typeOf.object('rotation', rotation);
        Check.typeOf.object('scale', scale);
        
        if (!defined(result)) {
            result = new Matrix4();
        }

        var scaleX = scale.x;
        var scaleY = scale.y;
        var scaleZ = scale.z;

        var x2 = rotation.x * rotation.x;
        var xy = rotation.x * rotation.y;
        var xz = rotation.x * rotation.z;
        var xw = rotation.x * rotation.w;
        var y2 = rotation.y * rotation.y;
        var yz = rotation.y * rotation.z;
        var yw = rotation.y * rotation.w;
        var z2 = rotation.z * rotation.z;
        var zw = rotation.z * rotation.w;
        var w2 = rotation.w * rotation.w;

        var m00 = x2 - y2 - z2 + w2;
        var m01 = 2.0 * (xy - zw);
        var m02 = 2.0 * (xz + yw);

        var m10 = 2.0 * (xy + zw);
        var m11 = -x2 + y2 - z2 + w2;
        var m12 = 2.0 * (yz - xw);

        var m20 = 2.0 * (xz - yw);
        var m21 = 2.0 * (yz + xw);
        var m22 = -x2 - y2 + z2 + w2;

        result[0]  = m00 * scaleX;
        result[1]  = m10 * scaleX;
        result[2]  = m20 * scaleX;
        result[3]  = 0.0;
        result[4]  = m01 * scaleY;
        result[5]  = m11 * scaleY;
        result[6]  = m21 * scaleY;
        result[7]  = 0.0;
        result[8]  = m02 * scaleZ;
        result[9]  = m12 * scaleZ;
        result[10] = m22 * scaleZ;
        result[11] = 0.0;
        result[12] = translation.x;
        result[13] = translation.y;
        result[14] = translation.z;
        result[15] = 1.0;

        return result;
    };

    /**
     * Creates a Matrix4 instance from a {@link TranslationRotationScale} instance.
     *
     * @param {TranslationRotationScale} translationRotationScale The instance.
     * @param {Matrix4} [result] The object in which the result will be stored, if undefined a new instance will be created.
     * @returns {Matrix4} The modified result parameter, or a new Matrix4 instance if one was not provided.
     */
    Matrix4.fromTranslationRotationScale = function(translationRotationScale, result) {
                Check.typeOf.object('translationRotationScale', translationRotationScale);
        
        return Matrix4.fromTranslationQuaternionRotationScale(translationRotationScale.translation, translationRotationScale.rotation, translationRotationScale.scale, result);
    };

    /**
     * Creates a Matrix4 instance from a Cartesian3 representing the translation.
     *
     * @param {Cartesian3} translation The upper right portion of the matrix representing the translation.
     * @param {Matrix4} [result] The object in which the result will be stored, if undefined a new instance will be created.
     * @returns {Matrix4} The modified result parameter, or a new Matrix4 instance if one was not provided.
     *
     * @see Matrix4.multiplyByTranslation
     */
    Matrix4.fromTranslation = function(translation, result) {
                Check.typeOf.object('translation', translation);
        
        return Matrix4.fromRotationTranslation(Matrix3.IDENTITY, translation, result);
    };

    /**
     * Computes a Matrix4 instance representing a non-uniform scale.
     *
     * @param {Cartesian3} scale The x, y, and z scale factors.
     * @param {Matrix4} [result] The object in which the result will be stored, if undefined a new instance will be created.
     * @returns {Matrix4} The modified result parameter, or a new Matrix4 instance if one was not provided.
     *
     * @example
     * // Creates
     * //   [7.0, 0.0, 0.0, 0.0]
     * //   [0.0, 8.0, 0.0, 0.0]
     * //   [0.0, 0.0, 9.0, 0.0]
     * //   [0.0, 0.0, 0.0, 1.0]
     * var m = Cesium.Matrix4.fromScale(new Cesium.Cartesian3(7.0, 8.0, 9.0));
     */
    Matrix4.fromScale = function(scale, result) {
                Check.typeOf.object('scale', scale);
        
        if (!defined(result)) {
            return new Matrix4(
                scale.x, 0.0,     0.0,     0.0,
                0.0,     scale.y, 0.0,     0.0,
                0.0,     0.0,     scale.z, 0.0,
                0.0,     0.0,     0.0,     1.0);
        }

        result[0] = scale.x;
        result[1] = 0.0;
        result[2] = 0.0;
        result[3] = 0.0;
        result[4] = 0.0;
        result[5] = scale.y;
        result[6] = 0.0;
        result[7] = 0.0;
        result[8] = 0.0;
        result[9] = 0.0;
        result[10] = scale.z;
        result[11] = 0.0;
        result[12] = 0.0;
        result[13] = 0.0;
        result[14] = 0.0;
        result[15] = 1.0;
        return result;
    };

    /**
     * Computes a Matrix4 instance representing a uniform scale.
     *
     * @param {Number} scale The uniform scale factor.
     * @param {Matrix4} [result] The object in which the result will be stored, if undefined a new instance will be created.
     * @returns {Matrix4} The modified result parameter, or a new Matrix4 instance if one was not provided.
     *
     * @example
     * // Creates
     * //   [2.0, 0.0, 0.0, 0.0]
     * //   [0.0, 2.0, 0.0, 0.0]
     * //   [0.0, 0.0, 2.0, 0.0]
     * //   [0.0, 0.0, 0.0, 1.0]
     * var m = Cesium.Matrix4.fromUniformScale(2.0);
     */
    Matrix4.fromUniformScale = function(scale, result) {
                Check.typeOf.number('scale', scale);
        
        if (!defined(result)) {
            return new Matrix4(scale, 0.0,   0.0,   0.0,
                               0.0,   scale, 0.0,   0.0,
                               0.0,   0.0,   scale, 0.0,
                               0.0,   0.0,   0.0,   1.0);
        }

        result[0] = scale;
        result[1] = 0.0;
        result[2] = 0.0;
        result[3] = 0.0;
        result[4] = 0.0;
        result[5] = scale;
        result[6] = 0.0;
        result[7] = 0.0;
        result[8] = 0.0;
        result[9] = 0.0;
        result[10] = scale;
        result[11] = 0.0;
        result[12] = 0.0;
        result[13] = 0.0;
        result[14] = 0.0;
        result[15] = 1.0;
        return result;
    };

    var fromCameraF = new Cartesian3();
    var fromCameraR = new Cartesian3();
    var fromCameraU = new Cartesian3();

    /**
     * Computes a Matrix4 instance from a Camera.
     *
     * @param {Camera} camera The camera to use.
     * @param {Matrix4} [result] The object in which the result will be stored, if undefined a new instance will be created.
     * @returns {Matrix4} The modified result parameter, or a new Matrix4 instance if one was not provided.
     */
    Matrix4.fromCamera = function(camera, result) {
                Check.typeOf.object('camera', camera);
        
        var position = camera.position;
        var direction = camera.direction;
        var up = camera.up;

                Check.typeOf.object('camera.position', position);
        Check.typeOf.object('camera.direction', direction);
        Check.typeOf.object('camera.up', up);
        
        Cartesian3.normalize(direction, fromCameraF);
        Cartesian3.normalize(Cartesian3.cross(fromCameraF, up, fromCameraR), fromCameraR);
        Cartesian3.normalize(Cartesian3.cross(fromCameraR, fromCameraF, fromCameraU), fromCameraU);

        var sX = fromCameraR.x;
        var sY = fromCameraR.y;
        var sZ = fromCameraR.z;
        var fX = fromCameraF.x;
        var fY = fromCameraF.y;
        var fZ = fromCameraF.z;
        var uX = fromCameraU.x;
        var uY = fromCameraU.y;
        var uZ = fromCameraU.z;
        var positionX = position.x;
        var positionY = position.y;
        var positionZ = position.z;
        var t0 = sX * -positionX + sY * -positionY+ sZ * -positionZ;
        var t1 = uX * -positionX + uY * -positionY+ uZ * -positionZ;
        var t2 = fX * positionX + fY * positionY + fZ * positionZ;

        // The code below this comment is an optimized
        // version of the commented lines.
        // Rather that create two matrices and then multiply,
        // we just bake in the multiplcation as part of creation.
        // var rotation = new Matrix4(
        //                 sX,  sY,  sZ, 0.0,
        //                 uX,  uY,  uZ, 0.0,
        //                -fX, -fY, -fZ, 0.0,
        //                 0.0,  0.0,  0.0, 1.0);
        // var translation = new Matrix4(
        //                 1.0, 0.0, 0.0, -position.x,
        //                 0.0, 1.0, 0.0, -position.y,
        //                 0.0, 0.0, 1.0, -position.z,
        //                 0.0, 0.0, 0.0, 1.0);
        // return rotation.multiply(translation);
        if (!defined(result)) {
            return new Matrix4(
                    sX,   sY,  sZ, t0,
                    uX,   uY,  uZ, t1,
                   -fX,  -fY, -fZ, t2,
                    0.0, 0.0, 0.0, 1.0);
        }
        result[0] = sX;
        result[1] = uX;
        result[2] = -fX;
        result[3] = 0.0;
        result[4] = sY;
        result[5] = uY;
        result[6] = -fY;
        result[7] = 0.0;
        result[8] = sZ;
        result[9] = uZ;
        result[10] = -fZ;
        result[11] = 0.0;
        result[12] = t0;
        result[13] = t1;
        result[14] = t2;
        result[15] = 1.0;
        return result;
    };

     /**
      * Computes a Matrix4 instance representing a perspective transformation matrix.
      *
      * @param {Number} fovY The field of view along the Y axis in radians.
      * @param {Number} aspectRatio The aspect ratio.
      * @param {Number} near The distance to the near plane in meters.
      * @param {Number} far The distance to the far plane in meters.
      * @param {Matrix4} result The object in which the result will be stored.
      * @returns {Matrix4} The modified result parameter.
      *
      * @exception {DeveloperError} fovY must be in (0, PI].
      * @exception {DeveloperError} aspectRatio must be greater than zero.
      * @exception {DeveloperError} near must be greater than zero.
      * @exception {DeveloperError} far must be greater than zero.
      */
    Matrix4.computePerspectiveFieldOfView = function(fovY, aspectRatio, near, far, result) {
                Check.typeOf.number.greaterThan('fovY', fovY, 0.0);
        Check.typeOf.number.lessThan('fovY', fovY, Math.PI);
        Check.typeOf.number.greaterThan('near', near, 0.0);
        Check.typeOf.number.greaterThan('far', far, 0.0);
        Check.typeOf.object('result', result);
        
        var bottom = Math.tan(fovY * 0.5);

        var column1Row1 = 1.0 / bottom;
        var column0Row0 = column1Row1 / aspectRatio;
        var column2Row2 = (far + near) / (near - far);
        var column3Row2 = (2.0 * far * near) / (near - far);

        result[0] = column0Row0;
        result[1] = 0.0;
        result[2] = 0.0;
        result[3] = 0.0;
        result[4] = 0.0;
        result[5] = column1Row1;
        result[6] = 0.0;
        result[7] = 0.0;
        result[8] = 0.0;
        result[9] = 0.0;
        result[10] = column2Row2;
        result[11] = -1.0;
        result[12] = 0.0;
        result[13] = 0.0;
        result[14] = column3Row2;
        result[15] = 0.0;
        return result;
    };

    /**
    * Computes a Matrix4 instance representing an orthographic transformation matrix.
    *
    * @param {Number} left The number of meters to the left of the camera that will be in view.
    * @param {Number} right The number of meters to the right of the camera that will be in view.
    * @param {Number} bottom The number of meters below of the camera that will be in view.
    * @param {Number} top The number of meters above of the camera that will be in view.
    * @param {Number} near The distance to the near plane in meters.
    * @param {Number} far The distance to the far plane in meters.
    * @param {Matrix4} result The object in which the result will be stored.
    * @returns {Matrix4} The modified result parameter.
    */
    Matrix4.computeOrthographicOffCenter = function(left, right, bottom, top, near, far, result) {
                Check.typeOf.number('left', left);
        Check.typeOf.number('right', right);
        Check.typeOf.number('bottom', bottom);
        Check.typeOf.number('top', top);
        Check.typeOf.number('near', near);
        Check.typeOf.number('far', far);
        Check.typeOf.object('result', result);
        
        var a = 1.0 / (right - left);
        var b = 1.0 / (top - bottom);
        var c = 1.0 / (far - near);

        var tx = -(right + left) * a;
        var ty = -(top + bottom) * b;
        var tz = -(far + near) * c;
        a *= 2.0;
        b *= 2.0;
        c *= -2.0;

        result[0] = a;
        result[1] = 0.0;
        result[2] = 0.0;
        result[3] = 0.0;
        result[4] = 0.0;
        result[5] = b;
        result[6] = 0.0;
        result[7] = 0.0;
        result[8] = 0.0;
        result[9] = 0.0;
        result[10] = c;
        result[11] = 0.0;
        result[12] = tx;
        result[13] = ty;
        result[14] = tz;
        result[15] = 1.0;
        return result;
    };

    /**
     * Computes a Matrix4 instance representing an off center perspective transformation.
     *
     * @param {Number} left The number of meters to the left of the camera that will be in view.
     * @param {Number} right The number of meters to the right of the camera that will be in view.
     * @param {Number} bottom The number of meters below of the camera that will be in view.
     * @param {Number} top The number of meters above of the camera that will be in view.
     * @param {Number} near The distance to the near plane in meters.
     * @param {Number} far The distance to the far plane in meters.
     * @param {Matrix4} result The object in which the result will be stored.
     * @returns {Matrix4} The modified result parameter.
     */
    Matrix4.computePerspectiveOffCenter = function(left, right, bottom, top, near, far, result) {
                Check.typeOf.number('left', left);
        Check.typeOf.number('right', right);
        Check.typeOf.number('bottom', bottom);
        Check.typeOf.number('top', top);
        Check.typeOf.number('near', near);
        Check.typeOf.number('far', far);
        Check.typeOf.object('result', result);
        
        var column0Row0 = 2.0 * near / (right - left);
        var column1Row1 = 2.0 * near / (top - bottom);
        var column2Row0 = (right + left) / (right - left);
        var column2Row1 = (top + bottom) / (top - bottom);
        var column2Row2 = -(far + near) / (far - near);
        var column2Row3 = -1.0;
        var column3Row2 = -2.0 * far * near / (far - near);

        result[0] = column0Row0;
        result[1] = 0.0;
        result[2] = 0.0;
        result[3] = 0.0;
        result[4] = 0.0;
        result[5] = column1Row1;
        result[6] = 0.0;
        result[7] = 0.0;
        result[8] = column2Row0;
        result[9] = column2Row1;
        result[10] = column2Row2;
        result[11] = column2Row3;
        result[12] = 0.0;
        result[13] = 0.0;
        result[14] = column3Row2;
        result[15] = 0.0;
        return result;
    };

    /**
     * Computes a Matrix4 instance representing an infinite off center perspective transformation.
     *
     * @param {Number} left The number of meters to the left of the camera that will be in view.
     * @param {Number} right The number of meters to the right of the camera that will be in view.
     * @param {Number} bottom The number of meters below of the camera that will be in view.
     * @param {Number} top The number of meters above of the camera that will be in view.
     * @param {Number} near The distance to the near plane in meters.
     * @param {Matrix4} result The object in which the result will be stored.
     * @returns {Matrix4} The modified result parameter.
     */
    Matrix4.computeInfinitePerspectiveOffCenter = function(left, right, bottom, top, near, result) {
                Check.typeOf.number('left', left);
        Check.typeOf.number('right', right);
        Check.typeOf.number('bottom', bottom);
        Check.typeOf.number('top', top);
        Check.typeOf.number('near', near);
        Check.typeOf.object('result', result);
        
        var column0Row0 = 2.0 * near / (right - left);
        var column1Row1 = 2.0 * near / (top - bottom);
        var column2Row0 = (right + left) / (right - left);
        var column2Row1 = (top + bottom) / (top - bottom);
        var column2Row2 = -1.0;
        var column2Row3 = -1.0;
        var column3Row2 = -2.0 * near;

        result[0] = column0Row0;
        result[1] = 0.0;
        result[2] = 0.0;
        result[3] = 0.0;
        result[4] = 0.0;
        result[5] = column1Row1;
        result[6] = 0.0;
        result[7] = 0.0;
        result[8] = column2Row0;
        result[9] = column2Row1;
        result[10] = column2Row2;
        result[11] = column2Row3;
        result[12] = 0.0;
        result[13] = 0.0;
        result[14] = column3Row2;
        result[15] = 0.0;
        return result;
    };

    /**
     * Computes a Matrix4 instance that transforms from normalized device coordinates to window coordinates.
     *
     * @param {Object}[viewport = { x : 0.0, y : 0.0, width : 0.0, height : 0.0 }] The viewport's corners as shown in Example 1.
     * @param {Number}[nearDepthRange=0.0] The near plane distance in window coordinates.
     * @param {Number}[farDepthRange=1.0] The far plane distance in window coordinates.
     * @param {Matrix4} result The object in which the result will be stored.
     * @returns {Matrix4} The modified result parameter.
     *
     * @example
     * // Create viewport transformation using an explicit viewport and depth range.
     * var m = Cesium.Matrix4.computeViewportTransformation({
     *     x : 0.0,
     *     y : 0.0,
     *     width : 1024.0,
     *     height : 768.0
     * }, 0.0, 1.0, new Cesium.Matrix4());
     */
    Matrix4.computeViewportTransformation = function(viewport, nearDepthRange, farDepthRange, result) {
                Check.typeOf.object('result', result);
        
        viewport = defaultValue(viewport, defaultValue.EMPTY_OBJECT);
        var x = defaultValue(viewport.x, 0.0);
        var y = defaultValue(viewport.y, 0.0);
        var width = defaultValue(viewport.width, 0.0);
        var height = defaultValue(viewport.height, 0.0);
        nearDepthRange = defaultValue(nearDepthRange, 0.0);
        farDepthRange = defaultValue(farDepthRange, 1.0);

        var halfWidth = width * 0.5;
        var halfHeight = height * 0.5;
        var halfDepth = (farDepthRange - nearDepthRange) * 0.5;

        var column0Row0 = halfWidth;
        var column1Row1 = halfHeight;
        var column2Row2 = halfDepth;
        var column3Row0 = x + halfWidth;
        var column3Row1 = y + halfHeight;
        var column3Row2 = nearDepthRange + halfDepth;
        var column3Row3 = 1.0;

        result[0] = column0Row0;
        result[1] = 0.0;
        result[2] = 0.0;
        result[3] = 0.0;
        result[4] = 0.0;
        result[5] = column1Row1;
        result[6] = 0.0;
        result[7] = 0.0;
        result[8] = 0.0;
        result[9] = 0.0;
        result[10] = column2Row2;
        result[11] = 0.0;
        result[12] = column3Row0;
        result[13] = column3Row1;
        result[14] = column3Row2;
        result[15] = column3Row3;
        return result;
    };

    /**
     * Computes a Matrix4 instance that transforms from world space to view space.
     *
     * @param {Cartesian3} position The position of the camera.
     * @param {Cartesian3} direction The forward direction.
     * @param {Cartesian3} up The up direction.
     * @param {Cartesian3} right The right direction.
     * @param {Matrix4} result The object in which the result will be stored.
     * @returns {Matrix4} The modified result parameter.
     */
    Matrix4.computeView = function(position, direction, up, right, result) {
                Check.typeOf.object('position', position);
        Check.typeOf.object('direction', direction);
        Check.typeOf.object('up', up);
        Check.typeOf.object('right', right);
        Check.typeOf.object('result', result);
        
        result[0] = right.x;
        result[1] = up.x;
        result[2] = -direction.x;
        result[3] = 0.0;
        result[4] = right.y;
        result[5] = up.y;
        result[6] = -direction.y;
        result[7] = 0.0;
        result[8] = right.z;
        result[9] = up.z;
        result[10] = -direction.z;
        result[11] = 0.0;
        result[12] = -Cartesian3.dot(right, position);
        result[13] = -Cartesian3.dot(up, position);
        result[14] = Cartesian3.dot(direction, position);
        result[15] = 1.0;
        return result;
    };

    /**
     * Computes an Array from the provided Matrix4 instance.
     * The array will be in column-major order.
     *
     * @param {Matrix4} matrix The matrix to use..
     * @param {Number[]} [result] The Array onto which to store the result.
     * @returns {Number[]} The modified Array parameter or a new Array instance if one was not provided.
     *
     * @example
     * //create an array from an instance of Matrix4
     * // m = [10.0, 14.0, 18.0, 22.0]
     * //     [11.0, 15.0, 19.0, 23.0]
     * //     [12.0, 16.0, 20.0, 24.0]
     * //     [13.0, 17.0, 21.0, 25.0]
     * var a = Cesium.Matrix4.toArray(m);
     *
     * // m remains the same
     * //creates a = [10.0, 11.0, 12.0, 13.0, 14.0, 15.0, 16.0, 17.0, 18.0, 19.0, 20.0, 21.0, 22.0, 23.0, 24.0, 25.0]
     */
    Matrix4.toArray = function(matrix, result) {
                Check.typeOf.object('matrix', matrix);
        
        if (!defined(result)) {
            return [matrix[0], matrix[1], matrix[2], matrix[3],
                    matrix[4], matrix[5], matrix[6], matrix[7],
                    matrix[8], matrix[9], matrix[10], matrix[11],
                    matrix[12], matrix[13], matrix[14], matrix[15]];
        }
        result[0] = matrix[0];
        result[1] = matrix[1];
        result[2] = matrix[2];
        result[3] = matrix[3];
        result[4] = matrix[4];
        result[5] = matrix[5];
        result[6] = matrix[6];
        result[7] = matrix[7];
        result[8] = matrix[8];
        result[9] = matrix[9];
        result[10] = matrix[10];
        result[11] = matrix[11];
        result[12] = matrix[12];
        result[13] = matrix[13];
        result[14] = matrix[14];
        result[15] = matrix[15];
        return result;
    };

    /**
     * Computes the array index of the element at the provided row and column.
     *
     * @param {Number} row The zero-based index of the row.
     * @param {Number} column The zero-based index of the column.
     * @returns {Number} The index of the element at the provided row and column.
     *
     * @exception {DeveloperError} row must be 0, 1, 2, or 3.
     * @exception {DeveloperError} column must be 0, 1, 2, or 3.
     *
     * @example
     * var myMatrix = new Cesium.Matrix4();
     * var column1Row0Index = Cesium.Matrix4.getElementIndex(1, 0);
     * var column1Row0 = myMatrix[column1Row0Index];
     * myMatrix[column1Row0Index] = 10.0;
     */
    Matrix4.getElementIndex = function(column, row) {
                Check.typeOf.number.greaterThanOrEquals('row', row, 0);
        Check.typeOf.number.lessThanOrEquals('row', row, 3);

        Check.typeOf.number.greaterThanOrEquals('column', column, 0);
        Check.typeOf.number.lessThanOrEquals('column', column, 3);
        
        return column * 4 + row;
    };

    /**
     * Retrieves a copy of the matrix column at the provided index as a Cartesian4 instance.
     *
     * @param {Matrix4} matrix The matrix to use.
     * @param {Number} index The zero-based index of the column to retrieve.
     * @param {Cartesian4} result The object onto which to store the result.
     * @returns {Cartesian4} The modified result parameter.
     *
     * @exception {DeveloperError} index must be 0, 1, 2, or 3.
     *
     * @example
     * //returns a Cartesian4 instance with values from the specified column
     * // m = [10.0, 11.0, 12.0, 13.0]
     * //     [14.0, 15.0, 16.0, 17.0]
     * //     [18.0, 19.0, 20.0, 21.0]
     * //     [22.0, 23.0, 24.0, 25.0]
     *
     * //Example 1: Creates an instance of Cartesian
     * var a = Cesium.Matrix4.getColumn(m, 2, new Cesium.Cartesian4());
     *
     * @example
     * //Example 2: Sets values for Cartesian instance
     * var a = new Cesium.Cartesian4();
     * Cesium.Matrix4.getColumn(m, 2, a);
     *
     * // a.x = 12.0; a.y = 16.0; a.z = 20.0; a.w = 24.0;
     */
    Matrix4.getColumn = function(matrix, index, result) {
                Check.typeOf.object('matrix', matrix);

        Check.typeOf.number.greaterThanOrEquals('index', index, 0);
        Check.typeOf.number.lessThanOrEquals('index', index, 3);

        Check.typeOf.object('result', result);
        
        var startIndex = index * 4;
        var x = matrix[startIndex];
        var y = matrix[startIndex + 1];
        var z = matrix[startIndex + 2];
        var w = matrix[startIndex + 3];

        result.x = x;
        result.y = y;
        result.z = z;
        result.w = w;
        return result;
    };

    /**
     * Computes a new matrix that replaces the specified column in the provided matrix with the provided Cartesian4 instance.
     *
     * @param {Matrix4} matrix The matrix to use.
     * @param {Number} index The zero-based index of the column to set.
     * @param {Cartesian4} cartesian The Cartesian whose values will be assigned to the specified column.
     * @param {Matrix4} result The object onto which to store the result.
     * @returns {Matrix4} The modified result parameter.
     *
     * @exception {DeveloperError} index must be 0, 1, 2, or 3.
     *
     * @example
     * //creates a new Matrix4 instance with new column values from the Cartesian4 instance
     * // m = [10.0, 11.0, 12.0, 13.0]
     * //     [14.0, 15.0, 16.0, 17.0]
     * //     [18.0, 19.0, 20.0, 21.0]
     * //     [22.0, 23.0, 24.0, 25.0]
     *
     * var a = Cesium.Matrix4.setColumn(m, 2, new Cesium.Cartesian4(99.0, 98.0, 97.0, 96.0), new Cesium.Matrix4());
     *
     * // m remains the same
     * // a = [10.0, 11.0, 99.0, 13.0]
     * //     [14.0, 15.0, 98.0, 17.0]
     * //     [18.0, 19.0, 97.0, 21.0]
     * //     [22.0, 23.0, 96.0, 25.0]
     */
    Matrix4.setColumn = function(matrix, index, cartesian, result) {
                Check.typeOf.object('matrix', matrix);

        Check.typeOf.number.greaterThanOrEquals('index', index, 0);
        Check.typeOf.number.lessThanOrEquals('index', index, 3);

        Check.typeOf.object('cartesian', cartesian);
        Check.typeOf.object('result', result);
        
        result = Matrix4.clone(matrix, result);
        var startIndex = index * 4;
        result[startIndex] = cartesian.x;
        result[startIndex + 1] = cartesian.y;
        result[startIndex + 2] = cartesian.z;
        result[startIndex + 3] = cartesian.w;
        return result;
    };

    /**
     * Computes a new matrix that replaces the translation in the rightmost column of the provided
     * matrix with the provided translation.  This assumes the matrix is an affine transformation
     *
     * @param {Matrix4} matrix The matrix to use.
     * @param {Cartesian3} translation The translation that replaces the translation of the provided matrix.
     * @param {Matrix4} result The object onto which to store the result.
     * @returns {Matrix4} The modified result parameter.
     */
    Matrix4.setTranslation = function(matrix, translation, result) {
                Check.typeOf.object('matrix', matrix);
        Check.typeOf.object('translation', translation);
        Check.typeOf.object('result', result);
        
        result[0] = matrix[0];
        result[1] = matrix[1];
        result[2] = matrix[2];
        result[3] = matrix[3];

        result[4] = matrix[4];
        result[5] = matrix[5];
        result[6] = matrix[6];
        result[7] = matrix[7];

        result[8] = matrix[8];
        result[9] = matrix[9];
        result[10] = matrix[10];
        result[11] = matrix[11];

        result[12] = translation.x;
        result[13] = translation.y;
        result[14] = translation.z;
        result[15] = matrix[15];

        return result;
    };

    var scaleScratch = new Cartesian3();
    /**
     * Computes a new matrix that replaces the scale with the provided scale.  This assumes the matrix is an affine transformation
     *
     * @param {Matrix4} matrix The matrix to use.
     * @param {Cartesian3} scale The scale that replaces the scale of the provided matrix.
     * @param {Matrix4} result The object onto which to store the result.
     * @returns {Matrix4} The modified result parameter.
     */
    Matrix4.setScale = function(matrix, scale, result) {
                Check.typeOf.object('matrix', matrix);
        Check.typeOf.object('scale', scale);
        Check.typeOf.object('result', result);
        
        var existingScale = Matrix4.getScale(matrix, scaleScratch);
        var newScale = Cartesian3.divideComponents(scale, existingScale, scaleScratch);
        return Matrix4.multiplyByScale(matrix, newScale, result);
    };

    /**
     * Retrieves a copy of the matrix row at the provided index as a Cartesian4 instance.
     *
     * @param {Matrix4} matrix The matrix to use.
     * @param {Number} index The zero-based index of the row to retrieve.
     * @param {Cartesian4} result The object onto which to store the result.
     * @returns {Cartesian4} The modified result parameter.
     *
     * @exception {DeveloperError} index must be 0, 1, 2, or 3.
     *
     * @example
     * //returns a Cartesian4 instance with values from the specified column
     * // m = [10.0, 11.0, 12.0, 13.0]
     * //     [14.0, 15.0, 16.0, 17.0]
     * //     [18.0, 19.0, 20.0, 21.0]
     * //     [22.0, 23.0, 24.0, 25.0]
     *
     * //Example 1: Returns an instance of Cartesian
     * var a = Cesium.Matrix4.getRow(m, 2, new Cesium.Cartesian4());
     *
     * @example
     * //Example 2: Sets values for a Cartesian instance
     * var a = new Cesium.Cartesian4();
     * Cesium.Matrix4.getRow(m, 2, a);
     *
     * // a.x = 18.0; a.y = 19.0; a.z = 20.0; a.w = 21.0;
     */
    Matrix4.getRow = function(matrix, index, result) {
                Check.typeOf.object('matrix', matrix);

        Check.typeOf.number.greaterThanOrEquals('index', index, 0);
        Check.typeOf.number.lessThanOrEquals('index', index, 3);

        Check.typeOf.object('result', result);
        
        var x = matrix[index];
        var y = matrix[index + 4];
        var z = matrix[index + 8];
        var w = matrix[index + 12];

        result.x = x;
        result.y = y;
        result.z = z;
        result.w = w;
        return result;
    };

    /**
     * Computes a new matrix that replaces the specified row in the provided matrix with the provided Cartesian4 instance.
     *
     * @param {Matrix4} matrix The matrix to use.
     * @param {Number} index The zero-based index of the row to set.
     * @param {Cartesian4} cartesian The Cartesian whose values will be assigned to the specified row.
     * @param {Matrix4} result The object onto which to store the result.
     * @returns {Matrix4} The modified result parameter.
     *
     * @exception {DeveloperError} index must be 0, 1, 2, or 3.
     *
     * @example
     * //create a new Matrix4 instance with new row values from the Cartesian4 instance
     * // m = [10.0, 11.0, 12.0, 13.0]
     * //     [14.0, 15.0, 16.0, 17.0]
     * //     [18.0, 19.0, 20.0, 21.0]
     * //     [22.0, 23.0, 24.0, 25.0]
     *
     * var a = Cesium.Matrix4.setRow(m, 2, new Cesium.Cartesian4(99.0, 98.0, 97.0, 96.0), new Cesium.Matrix4());
     *
     * // m remains the same
     * // a = [10.0, 11.0, 12.0, 13.0]
     * //     [14.0, 15.0, 16.0, 17.0]
     * //     [99.0, 98.0, 97.0, 96.0]
     * //     [22.0, 23.0, 24.0, 25.0]
     */
    Matrix4.setRow = function(matrix, index, cartesian, result) {
                Check.typeOf.object('matrix', matrix);

        Check.typeOf.number.greaterThanOrEquals('index', index, 0);
        Check.typeOf.number.lessThanOrEquals('index', index, 3);

        Check.typeOf.object('cartesian', cartesian);
        Check.typeOf.object('result', result);
        
        result = Matrix4.clone(matrix, result);
        result[index] = cartesian.x;
        result[index + 4] = cartesian.y;
        result[index + 8] = cartesian.z;
        result[index + 12] = cartesian.w;
        return result;
    };

    var scratchColumn = new Cartesian3();

    /**
     * Extracts the non-uniform scale assuming the matrix is an affine transformation.
     *
     * @param {Matrix4} matrix The matrix.
     * @param {Cartesian3} result The object onto which to store the result.
     * @returns {Cartesian3} The modified result parameter
     */
    Matrix4.getScale = function(matrix, result) {
                Check.typeOf.object('matrix', matrix);
        Check.typeOf.object('result', result);
        
        result.x = Cartesian3.magnitude(Cartesian3.fromElements(matrix[0], matrix[1], matrix[2], scratchColumn));
        result.y = Cartesian3.magnitude(Cartesian3.fromElements(matrix[4], matrix[5], matrix[6], scratchColumn));
        result.z = Cartesian3.magnitude(Cartesian3.fromElements(matrix[8], matrix[9], matrix[10], scratchColumn));
        return result;
    };

    var scratchScale = new Cartesian3();

    /**
     * Computes the maximum scale assuming the matrix is an affine transformation.
     * The maximum scale is the maximum length of the column vectors in the upper-left
     * 3x3 matrix.
     *
     * @param {Matrix4} matrix The matrix.
     * @returns {Number} The maximum scale.
     */
    Matrix4.getMaximumScale = function(matrix) {
        Matrix4.getScale(matrix, scratchScale);
        return Cartesian3.maximumComponent(scratchScale);
    };

    /**
     * Computes the product of two matrices.
     *
     * @param {Matrix4} left The first matrix.
     * @param {Matrix4} right The second matrix.
     * @param {Matrix4} result The object onto which to store the result.
     * @returns {Matrix4} The modified result parameter.
     */
    Matrix4.multiply = function(left, right, result) {
                Check.typeOf.object('left', left);
        Check.typeOf.object('right', right);
        Check.typeOf.object('result', result);
        
        var left0 = left[0];
        var left1 = left[1];
        var left2 = left[2];
        var left3 = left[3];
        var left4 = left[4];
        var left5 = left[5];
        var left6 = left[6];
        var left7 = left[7];
        var left8 = left[8];
        var left9 = left[9];
        var left10 = left[10];
        var left11 = left[11];
        var left12 = left[12];
        var left13 = left[13];
        var left14 = left[14];
        var left15 = left[15];

        var right0 = right[0];
        var right1 = right[1];
        var right2 = right[2];
        var right3 = right[3];
        var right4 = right[4];
        var right5 = right[5];
        var right6 = right[6];
        var right7 = right[7];
        var right8 = right[8];
        var right9 = right[9];
        var right10 = right[10];
        var right11 = right[11];
        var right12 = right[12];
        var right13 = right[13];
        var right14 = right[14];
        var right15 = right[15];

        var column0Row0 = left0 * right0 + left4 * right1 + left8 * right2 + left12 * right3;
        var column0Row1 = left1 * right0 + left5 * right1 + left9 * right2 + left13 * right3;
        var column0Row2 = left2 * right0 + left6 * right1 + left10 * right2 + left14 * right3;
        var column0Row3 = left3 * right0 + left7 * right1 + left11 * right2 + left15 * right3;

        var column1Row0 = left0 * right4 + left4 * right5 + left8 * right6 + left12 * right7;
        var column1Row1 = left1 * right4 + left5 * right5 + left9 * right6 + left13 * right7;
        var column1Row2 = left2 * right4 + left6 * right5 + left10 * right6 + left14 * right7;
        var column1Row3 = left3 * right4 + left7 * right5 + left11 * right6 + left15 * right7;

        var column2Row0 = left0 * right8 + left4 * right9 + left8 * right10 + left12 * right11;
        var column2Row1 = left1 * right8 + left5 * right9 + left9 * right10 + left13 * right11;
        var column2Row2 = left2 * right8 + left6 * right9 + left10 * right10 + left14 * right11;
        var column2Row3 = left3 * right8 + left7 * right9 + left11 * right10 + left15 * right11;

        var column3Row0 = left0 * right12 + left4 * right13 + left8 * right14 + left12 * right15;
        var column3Row1 = left1 * right12 + left5 * right13 + left9 * right14 + left13 * right15;
        var column3Row2 = left2 * right12 + left6 * right13 + left10 * right14 + left14 * right15;
        var column3Row3 = left3 * right12 + left7 * right13 + left11 * right14 + left15 * right15;

        result[0] = column0Row0;
        result[1] = column0Row1;
        result[2] = column0Row2;
        result[3] = column0Row3;
        result[4] = column1Row0;
        result[5] = column1Row1;
        result[6] = column1Row2;
        result[7] = column1Row3;
        result[8] = column2Row0;
        result[9] = column2Row1;
        result[10] = column2Row2;
        result[11] = column2Row3;
        result[12] = column3Row0;
        result[13] = column3Row1;
        result[14] = column3Row2;
        result[15] = column3Row3;
        return result;
    };

    /**
     * Computes the sum of two matrices.
     *
     * @param {Matrix4} left The first matrix.
     * @param {Matrix4} right The second matrix.
     * @param {Matrix4} result The object onto which to store the result.
     * @returns {Matrix4} The modified result parameter.
     */
    Matrix4.add = function(left, right, result) {
                Check.typeOf.object('left', left);
        Check.typeOf.object('right', right);
        Check.typeOf.object('result', result);
        
        result[0] = left[0] + right[0];
        result[1] = left[1] + right[1];
        result[2] = left[2] + right[2];
        result[3] = left[3] + right[3];
        result[4] = left[4] + right[4];
        result[5] = left[5] + right[5];
        result[6] = left[6] + right[6];
        result[7] = left[7] + right[7];
        result[8] = left[8] + right[8];
        result[9] = left[9] + right[9];
        result[10] = left[10] + right[10];
        result[11] = left[11] + right[11];
        result[12] = left[12] + right[12];
        result[13] = left[13] + right[13];
        result[14] = left[14] + right[14];
        result[15] = left[15] + right[15];
        return result;
    };

    /**
     * Computes the difference of two matrices.
     *
     * @param {Matrix4} left The first matrix.
     * @param {Matrix4} right The second matrix.
     * @param {Matrix4} result The object onto which to store the result.
     * @returns {Matrix4} The modified result parameter.
     */
    Matrix4.subtract = function(left, right, result) {
                Check.typeOf.object('left', left);
        Check.typeOf.object('right', right);
        Check.typeOf.object('result', result);
        
        result[0] = left[0] - right[0];
        result[1] = left[1] - right[1];
        result[2] = left[2] - right[2];
        result[3] = left[3] - right[3];
        result[4] = left[4] - right[4];
        result[5] = left[5] - right[5];
        result[6] = left[6] - right[6];
        result[7] = left[7] - right[7];
        result[8] = left[8] - right[8];
        result[9] = left[9] - right[9];
        result[10] = left[10] - right[10];
        result[11] = left[11] - right[11];
        result[12] = left[12] - right[12];
        result[13] = left[13] - right[13];
        result[14] = left[14] - right[14];
        result[15] = left[15] - right[15];
        return result;
    };

    /**
     * Computes the product of two matrices assuming the matrices are
     * affine transformation matrices, where the upper left 3x3 elements
     * are a rotation matrix, and the upper three elements in the fourth
     * column are the translation.  The bottom row is assumed to be [0, 0, 0, 1].
     * The matrix is not verified to be in the proper form.
     * This method is faster than computing the product for general 4x4
     * matrices using {@link Matrix4.multiply}.
     *
     * @param {Matrix4} left The first matrix.
     * @param {Matrix4} right The second matrix.
     * @param {Matrix4} result The object onto which to store the result.
     * @returns {Matrix4} The modified result parameter.
     *
     * @example
     * var m1 = new Cesium.Matrix4(1.0, 6.0, 7.0, 0.0, 2.0, 5.0, 8.0, 0.0, 3.0, 4.0, 9.0, 0.0, 0.0, 0.0, 0.0, 1.0);
     * var m2 = Cesium.Transforms.eastNorthUpToFixedFrame(new Cesium.Cartesian3(1.0, 1.0, 1.0));
     * var m3 = Cesium.Matrix4.multiplyTransformation(m1, m2, new Cesium.Matrix4());
     */
    Matrix4.multiplyTransformation = function(left, right, result) {
                Check.typeOf.object('left', left);
        Check.typeOf.object('right', right);
        Check.typeOf.object('result', result);
        
        var left0 = left[0];
        var left1 = left[1];
        var left2 = left[2];
        var left4 = left[4];
        var left5 = left[5];
        var left6 = left[6];
        var left8 = left[8];
        var left9 = left[9];
        var left10 = left[10];
        var left12 = left[12];
        var left13 = left[13];
        var left14 = left[14];

        var right0 = right[0];
        var right1 = right[1];
        var right2 = right[2];
        var right4 = right[4];
        var right5 = right[5];
        var right6 = right[6];
        var right8 = right[8];
        var right9 = right[9];
        var right10 = right[10];
        var right12 = right[12];
        var right13 = right[13];
        var right14 = right[14];

        var column0Row0 = left0 * right0 + left4 * right1 + left8 * right2;
        var column0Row1 = left1 * right0 + left5 * right1 + left9 * right2;
        var column0Row2 = left2 * right0 + left6 * right1 + left10 * right2;

        var column1Row0 = left0 * right4 + left4 * right5 + left8 * right6;
        var column1Row1 = left1 * right4 + left5 * right5 + left9 * right6;
        var column1Row2 = left2 * right4 + left6 * right5 + left10 * right6;

        var column2Row0 = left0 * right8 + left4 * right9 + left8 * right10;
        var column2Row1 = left1 * right8 + left5 * right9 + left9 * right10;
        var column2Row2 = left2 * right8 + left6 * right9 + left10 * right10;

        var column3Row0 = left0 * right12 + left4 * right13 + left8 * right14 + left12;
        var column3Row1 = left1 * right12 + left5 * right13 + left9 * right14 + left13;
        var column3Row2 = left2 * right12 + left6 * right13 + left10 * right14 + left14;

        result[0] = column0Row0;
        result[1] = column0Row1;
        result[2] = column0Row2;
        result[3] = 0.0;
        result[4] = column1Row0;
        result[5] = column1Row1;
        result[6] = column1Row2;
        result[7] = 0.0;
        result[8] = column2Row0;
        result[9] = column2Row1;
        result[10] = column2Row2;
        result[11] = 0.0;
        result[12] = column3Row0;
        result[13] = column3Row1;
        result[14] = column3Row2;
        result[15] = 1.0;
        return result;
    };

    /**
     * Multiplies a transformation matrix (with a bottom row of <code>[0.0, 0.0, 0.0, 1.0]</code>)
     * by a 3x3 rotation matrix.  This is an optimization
     * for <code>Matrix4.multiply(m, Matrix4.fromRotationTranslation(rotation), m);</code> with less allocations and arithmetic operations.
     *
     * @param {Matrix4} matrix The matrix on the left-hand side.
     * @param {Matrix3} rotation The 3x3 rotation matrix on the right-hand side.
     * @param {Matrix4} result The object onto which to store the result.
     * @returns {Matrix4} The modified result parameter.
     *
     * @example
     * // Instead of Cesium.Matrix4.multiply(m, Cesium.Matrix4.fromRotationTranslation(rotation), m);
     * Cesium.Matrix4.multiplyByMatrix3(m, rotation, m);
     */
    Matrix4.multiplyByMatrix3 = function(matrix, rotation, result) {
                Check.typeOf.object('matrix', matrix);
        Check.typeOf.object('rotation', rotation);
        Check.typeOf.object('result', result);
        
        var left0 = matrix[0];
        var left1 = matrix[1];
        var left2 = matrix[2];
        var left4 = matrix[4];
        var left5 = matrix[5];
        var left6 = matrix[6];
        var left8 = matrix[8];
        var left9 = matrix[9];
        var left10 = matrix[10];

        var right0 = rotation[0];
        var right1 = rotation[1];
        var right2 = rotation[2];
        var right4 = rotation[3];
        var right5 = rotation[4];
        var right6 = rotation[5];
        var right8 = rotation[6];
        var right9 = rotation[7];
        var right10 = rotation[8];

        var column0Row0 = left0 * right0 + left4 * right1 + left8 * right2;
        var column0Row1 = left1 * right0 + left5 * right1 + left9 * right2;
        var column0Row2 = left2 * right0 + left6 * right1 + left10 * right2;

        var column1Row0 = left0 * right4 + left4 * right5 + left8 * right6;
        var column1Row1 = left1 * right4 + left5 * right5 + left9 * right6;
        var column1Row2 = left2 * right4 + left6 * right5 + left10 * right6;

        var column2Row0 = left0 * right8 + left4 * right9 + left8 * right10;
        var column2Row1 = left1 * right8 + left5 * right9 + left9 * right10;
        var column2Row2 = left2 * right8 + left6 * right9 + left10 * right10;

        result[0] = column0Row0;
        result[1] = column0Row1;
        result[2] = column0Row2;
        result[3] = 0.0;
        result[4] = column1Row0;
        result[5] = column1Row1;
        result[6] = column1Row2;
        result[7] = 0.0;
        result[8] = column2Row0;
        result[9] = column2Row1;
        result[10] = column2Row2;
        result[11] = 0.0;
        result[12] = matrix[12];
        result[13] = matrix[13];
        result[14] = matrix[14];
        result[15] = matrix[15];
        return result;
    };

    /**
     * Multiplies a transformation matrix (with a bottom row of <code>[0.0, 0.0, 0.0, 1.0]</code>)
     * by an implicit translation matrix defined by a {@link Cartesian3}.  This is an optimization
     * for <code>Matrix4.multiply(m, Matrix4.fromTranslation(position), m);</code> with less allocations and arithmetic operations.
     *
     * @param {Matrix4} matrix The matrix on the left-hand side.
     * @param {Cartesian3} translation The translation on the right-hand side.
     * @param {Matrix4} result The object onto which to store the result.
     * @returns {Matrix4} The modified result parameter.
     *
     * @example
     * // Instead of Cesium.Matrix4.multiply(m, Cesium.Matrix4.fromTranslation(position), m);
     * Cesium.Matrix4.multiplyByTranslation(m, position, m);
     */
    Matrix4.multiplyByTranslation = function(matrix, translation, result) {
                Check.typeOf.object('matrix', matrix);
        Check.typeOf.object('translation', translation);
        Check.typeOf.object('result', result);
        
        var x = translation.x;
        var y = translation.y;
        var z = translation.z;

        var tx = (x * matrix[0]) + (y * matrix[4]) + (z * matrix[8]) + matrix[12];
        var ty = (x * matrix[1]) + (y * matrix[5]) + (z * matrix[9]) + matrix[13];
        var tz = (x * matrix[2]) + (y * matrix[6]) + (z * matrix[10]) + matrix[14];

        result[0] = matrix[0];
        result[1] = matrix[1];
        result[2] = matrix[2];
        result[3] = matrix[3];
        result[4] = matrix[4];
        result[5] = matrix[5];
        result[6] = matrix[6];
        result[7] = matrix[7];
        result[8] = matrix[8];
        result[9] = matrix[9];
        result[10] = matrix[10];
        result[11] = matrix[11];
        result[12] = tx;
        result[13] = ty;
        result[14] = tz;
        result[15] = matrix[15];
        return result;
    };

    var uniformScaleScratch = new Cartesian3();

    /**
     * Multiplies an affine transformation matrix (with a bottom row of <code>[0.0, 0.0, 0.0, 1.0]</code>)
     * by an implicit uniform scale matrix.  This is an optimization
     * for <code>Matrix4.multiply(m, Matrix4.fromUniformScale(scale), m);</code>, where
     * <code>m</code> must be an affine matrix.
     * This function performs fewer allocations and arithmetic operations.
     *
     * @param {Matrix4} matrix The affine matrix on the left-hand side.
     * @param {Number} scale The uniform scale on the right-hand side.
     * @param {Matrix4} result The object onto which to store the result.
     * @returns {Matrix4} The modified result parameter.
     *
     *
     * @example
     * // Instead of Cesium.Matrix4.multiply(m, Cesium.Matrix4.fromUniformScale(scale), m);
     * Cesium.Matrix4.multiplyByUniformScale(m, scale, m);
     *
     * @see Matrix4.fromUniformScale
     * @see Matrix4.multiplyByScale
     */
    Matrix4.multiplyByUniformScale = function(matrix, scale, result) {
                Check.typeOf.object('matrix', matrix);
        Check.typeOf.number('scale', scale);
        Check.typeOf.object('result', result);
        
        uniformScaleScratch.x = scale;
        uniformScaleScratch.y = scale;
        uniformScaleScratch.z = scale;
        return Matrix4.multiplyByScale(matrix, uniformScaleScratch, result);
    };

    /**
     * Multiplies an affine transformation matrix (with a bottom row of <code>[0.0, 0.0, 0.0, 1.0]</code>)
     * by an implicit non-uniform scale matrix.  This is an optimization
     * for <code>Matrix4.multiply(m, Matrix4.fromUniformScale(scale), m);</code>, where
     * <code>m</code> must be an affine matrix.
     * This function performs fewer allocations and arithmetic operations.
     *
     * @param {Matrix4} matrix The affine matrix on the left-hand side.
     * @param {Cartesian3} scale The non-uniform scale on the right-hand side.
     * @param {Matrix4} result The object onto which to store the result.
     * @returns {Matrix4} The modified result parameter.
     *
     *
     * @example
     * // Instead of Cesium.Matrix4.multiply(m, Cesium.Matrix4.fromScale(scale), m);
     * Cesium.Matrix4.multiplyByScale(m, scale, m);
     *
     * @see Matrix4.fromScale
     * @see Matrix4.multiplyByUniformScale
     */
    Matrix4.multiplyByScale = function(matrix, scale, result) {
                Check.typeOf.object('matrix', matrix);
        Check.typeOf.object('scale', scale);
        Check.typeOf.object('result', result);
        
        var scaleX = scale.x;
        var scaleY = scale.y;
        var scaleZ = scale.z;

        // Faster than Cartesian3.equals
        if ((scaleX === 1.0) && (scaleY === 1.0) && (scaleZ === 1.0)) {
            return Matrix4.clone(matrix, result);
        }

        result[0] = scaleX * matrix[0];
        result[1] = scaleX * matrix[1];
        result[2] = scaleX * matrix[2];
        result[3] = 0.0;
        result[4] = scaleY * matrix[4];
        result[5] = scaleY * matrix[5];
        result[6] = scaleY * matrix[6];
        result[7] = 0.0;
        result[8] = scaleZ * matrix[8];
        result[9] = scaleZ * matrix[9];
        result[10] = scaleZ * matrix[10];
        result[11] = 0.0;
        result[12] = matrix[12];
        result[13] = matrix[13];
        result[14] = matrix[14];
        result[15] = 1.0;
        return result;
    };

    /**
     * Computes the product of a matrix and a column vector.
     *
     * @param {Matrix4} matrix The matrix.
     * @param {Cartesian4} cartesian The vector.
     * @param {Cartesian4} result The object onto which to store the result.
     * @returns {Cartesian4} The modified result parameter.
     */
    Matrix4.multiplyByVector = function(matrix, cartesian, result) {
                Check.typeOf.object('matrix', matrix);
        Check.typeOf.object('cartesian', cartesian);
        Check.typeOf.object('result', result);
        
        var vX = cartesian.x;
        var vY = cartesian.y;
        var vZ = cartesian.z;
        var vW = cartesian.w;

        var x = matrix[0] * vX + matrix[4] * vY + matrix[8] * vZ + matrix[12] * vW;
        var y = matrix[1] * vX + matrix[5] * vY + matrix[9] * vZ + matrix[13] * vW;
        var z = matrix[2] * vX + matrix[6] * vY + matrix[10] * vZ + matrix[14] * vW;
        var w = matrix[3] * vX + matrix[7] * vY + matrix[11] * vZ + matrix[15] * vW;

        result.x = x;
        result.y = y;
        result.z = z;
        result.w = w;
        return result;
    };

    /**
     * Computes the product of a matrix and a {@link Cartesian3}.  This is equivalent to calling {@link Matrix4.multiplyByVector}
     * with a {@link Cartesian4} with a <code>w</code> component of zero.
     *
     * @param {Matrix4} matrix The matrix.
     * @param {Cartesian3} cartesian The point.
     * @param {Cartesian3} result The object onto which to store the result.
     * @returns {Cartesian3} The modified result parameter.
     *
     * @example
     * var p = new Cesium.Cartesian3(1.0, 2.0, 3.0);
     * var result = Cesium.Matrix4.multiplyByPointAsVector(matrix, p, new Cesium.Cartesian3());
     * // A shortcut for
     * //   Cartesian3 p = ...
     * //   Cesium.Matrix4.multiplyByVector(matrix, new Cesium.Cartesian4(p.x, p.y, p.z, 0.0), result);
     */
    Matrix4.multiplyByPointAsVector = function(matrix, cartesian, result) {
                Check.typeOf.object('matrix', matrix);
        Check.typeOf.object('cartesian', cartesian);
        Check.typeOf.object('result', result);
        
        var vX = cartesian.x;
        var vY = cartesian.y;
        var vZ = cartesian.z;

        var x = matrix[0] * vX + matrix[4] * vY + matrix[8] * vZ;
        var y = matrix[1] * vX + matrix[5] * vY + matrix[9] * vZ;
        var z = matrix[2] * vX + matrix[6] * vY + matrix[10] * vZ;

        result.x = x;
        result.y = y;
        result.z = z;
        return result;
    };

    /**
     * Computes the product of a matrix and a {@link Cartesian3}. This is equivalent to calling {@link Matrix4.multiplyByVector}
     * with a {@link Cartesian4} with a <code>w</code> component of 1, but returns a {@link Cartesian3} instead of a {@link Cartesian4}.
     *
     * @param {Matrix4} matrix The matrix.
     * @param {Cartesian3} cartesian The point.
     * @param {Cartesian3} result The object onto which to store the result.
     * @returns {Cartesian3} The modified result parameter.
     *
     * @example
     * var p = new Cesium.Cartesian3(1.0, 2.0, 3.0);
     * var result = Cesium.Matrix4.multiplyByPoint(matrix, p, new Cesium.Cartesian3());
     */
    Matrix4.multiplyByPoint = function(matrix, cartesian, result) {
                Check.typeOf.object('matrix', matrix);
        Check.typeOf.object('cartesian', cartesian);
        Check.typeOf.object('result', result);
        
        var vX = cartesian.x;
        var vY = cartesian.y;
        var vZ = cartesian.z;

        var x = matrix[0] * vX + matrix[4] * vY + matrix[8] * vZ + matrix[12];
        var y = matrix[1] * vX + matrix[5] * vY + matrix[9] * vZ + matrix[13];
        var z = matrix[2] * vX + matrix[6] * vY + matrix[10] * vZ + matrix[14];

        result.x = x;
        result.y = y;
        result.z = z;
        return result;
    };

    /**
     * Computes the product of a matrix and a scalar.
     *
     * @param {Matrix4} matrix The matrix.
     * @param {Number} scalar The number to multiply by.
     * @param {Matrix4} result The object onto which to store the result.
     * @returns {Matrix4} The modified result parameter.
     *
     * @example
     * //create a Matrix4 instance which is a scaled version of the supplied Matrix4
     * // m = [10.0, 11.0, 12.0, 13.0]
     * //     [14.0, 15.0, 16.0, 17.0]
     * //     [18.0, 19.0, 20.0, 21.0]
     * //     [22.0, 23.0, 24.0, 25.0]
     *
     * var a = Cesium.Matrix4.multiplyByScalar(m, -2, new Cesium.Matrix4());
     *
     * // m remains the same
     * // a = [-20.0, -22.0, -24.0, -26.0]
     * //     [-28.0, -30.0, -32.0, -34.0]
     * //     [-36.0, -38.0, -40.0, -42.0]
     * //     [-44.0, -46.0, -48.0, -50.0]
     */
    Matrix4.multiplyByScalar = function(matrix, scalar, result) {
                Check.typeOf.object('matrix', matrix);
        Check.typeOf.number('scalar', scalar);
        Check.typeOf.object('result', result);
        
        result[0] = matrix[0] * scalar;
        result[1] = matrix[1] * scalar;
        result[2] = matrix[2] * scalar;
        result[3] = matrix[3] * scalar;
        result[4] = matrix[4] * scalar;
        result[5] = matrix[5] * scalar;
        result[6] = matrix[6] * scalar;
        result[7] = matrix[7] * scalar;
        result[8] = matrix[8] * scalar;
        result[9] = matrix[9] * scalar;
        result[10] = matrix[10] * scalar;
        result[11] = matrix[11] * scalar;
        result[12] = matrix[12] * scalar;
        result[13] = matrix[13] * scalar;
        result[14] = matrix[14] * scalar;
        result[15] = matrix[15] * scalar;
        return result;
    };

    /**
     * Computes a negated copy of the provided matrix.
     *
     * @param {Matrix4} matrix The matrix to negate.
     * @param {Matrix4} result The object onto which to store the result.
     * @returns {Matrix4} The modified result parameter.
     *
     * @example
     * //create a new Matrix4 instance which is a negation of a Matrix4
     * // m = [10.0, 11.0, 12.0, 13.0]
     * //     [14.0, 15.0, 16.0, 17.0]
     * //     [18.0, 19.0, 20.0, 21.0]
     * //     [22.0, 23.0, 24.0, 25.0]
     *
     * var a = Cesium.Matrix4.negate(m, new Cesium.Matrix4());
     *
     * // m remains the same
     * // a = [-10.0, -11.0, -12.0, -13.0]
     * //     [-14.0, -15.0, -16.0, -17.0]
     * //     [-18.0, -19.0, -20.0, -21.0]
     * //     [-22.0, -23.0, -24.0, -25.0]
     */
    Matrix4.negate = function(matrix, result) {
                Check.typeOf.object('matrix', matrix);
        Check.typeOf.object('result', result);
        
        result[0] = -matrix[0];
        result[1] = -matrix[1];
        result[2] = -matrix[2];
        result[3] = -matrix[3];
        result[4] = -matrix[4];
        result[5] = -matrix[5];
        result[6] = -matrix[6];
        result[7] = -matrix[7];
        result[8] = -matrix[8];
        result[9] = -matrix[9];
        result[10] = -matrix[10];
        result[11] = -matrix[11];
        result[12] = -matrix[12];
        result[13] = -matrix[13];
        result[14] = -matrix[14];
        result[15] = -matrix[15];
        return result;
    };

    /**
     * Computes the transpose of the provided matrix.
     *
     * @param {Matrix4} matrix The matrix to transpose.
     * @param {Matrix4} result The object onto which to store the result.
     * @returns {Matrix4} The modified result parameter.
     *
     * @example
     * //returns transpose of a Matrix4
     * // m = [10.0, 11.0, 12.0, 13.0]
     * //     [14.0, 15.0, 16.0, 17.0]
     * //     [18.0, 19.0, 20.0, 21.0]
     * //     [22.0, 23.0, 24.0, 25.0]
     *
     * var a = Cesium.Matrix4.transpose(m, new Cesium.Matrix4());
     *
     * // m remains the same
     * // a = [10.0, 14.0, 18.0, 22.0]
     * //     [11.0, 15.0, 19.0, 23.0]
     * //     [12.0, 16.0, 20.0, 24.0]
     * //     [13.0, 17.0, 21.0, 25.0]
     */
    Matrix4.transpose = function(matrix, result) {
                Check.typeOf.object('matrix', matrix);
        Check.typeOf.object('result', result);
        
        var matrix1 = matrix[1];
        var matrix2 = matrix[2];
        var matrix3 = matrix[3];
        var matrix6 = matrix[6];
        var matrix7 = matrix[7];
        var matrix11 = matrix[11];

        result[0] = matrix[0];
        result[1] = matrix[4];
        result[2] = matrix[8];
        result[3] = matrix[12];
        result[4] = matrix1;
        result[5] = matrix[5];
        result[6] = matrix[9];
        result[7] = matrix[13];
        result[8] = matrix2;
        result[9] = matrix6;
        result[10] = matrix[10];
        result[11] = matrix[14];
        result[12] = matrix3;
        result[13] = matrix7;
        result[14] = matrix11;
        result[15] = matrix[15];
        return result;
    };

    /**
     * Computes a matrix, which contains the absolute (unsigned) values of the provided matrix's elements.
     *
     * @param {Matrix4} matrix The matrix with signed elements.
     * @param {Matrix4} result The object onto which to store the result.
     * @returns {Matrix4} The modified result parameter.
     */
    Matrix4.abs = function(matrix, result) {
                Check.typeOf.object('matrix', matrix);
        Check.typeOf.object('result', result);
        
        result[0] = Math.abs(matrix[0]);
        result[1] = Math.abs(matrix[1]);
        result[2] = Math.abs(matrix[2]);
        result[3] = Math.abs(matrix[3]);
        result[4] = Math.abs(matrix[4]);
        result[5] = Math.abs(matrix[5]);
        result[6] = Math.abs(matrix[6]);
        result[7] = Math.abs(matrix[7]);
        result[8] = Math.abs(matrix[8]);
        result[9] = Math.abs(matrix[9]);
        result[10] = Math.abs(matrix[10]);
        result[11] = Math.abs(matrix[11]);
        result[12] = Math.abs(matrix[12]);
        result[13] = Math.abs(matrix[13]);
        result[14] = Math.abs(matrix[14]);
        result[15] = Math.abs(matrix[15]);

        return result;
    };

    /**
     * Compares the provided matrices componentwise and returns
     * <code>true</code> if they are equal, <code>false</code> otherwise.
     *
     * @param {Matrix4} [left] The first matrix.
     * @param {Matrix4} [right] The second matrix.
     * @returns {Boolean} <code>true</code> if left and right are equal, <code>false</code> otherwise.
     *
     * @example
     * //compares two Matrix4 instances
     *
     * // a = [10.0, 14.0, 18.0, 22.0]
     * //     [11.0, 15.0, 19.0, 23.0]
     * //     [12.0, 16.0, 20.0, 24.0]
     * //     [13.0, 17.0, 21.0, 25.0]
     *
     * // b = [10.0, 14.0, 18.0, 22.0]
     * //     [11.0, 15.0, 19.0, 23.0]
     * //     [12.0, 16.0, 20.0, 24.0]
     * //     [13.0, 17.0, 21.0, 25.0]
     *
     * if(Cesium.Matrix4.equals(a,b)) {
     *      console.log("Both matrices are equal");
     * } else {
     *      console.log("They are not equal");
     * }
     *
     * //Prints "Both matrices are equal" on the console
     */
    Matrix4.equals = function(left, right) {
        // Given that most matrices will be transformation matrices, the elements
        // are tested in order such that the test is likely to fail as early
        // as possible.  I _think_ this is just as friendly to the L1 cache
        // as testing in index order.  It is certainty faster in practice.
        return (left === right) ||
               (defined(left) &&
                defined(right) &&
                // Translation
                left[12] === right[12] &&
                left[13] === right[13] &&
                left[14] === right[14] &&

                // Rotation/scale
                left[0] === right[0] &&
                left[1] === right[1] &&
                left[2] === right[2] &&
                left[4] === right[4] &&
                left[5] === right[5] &&
                left[6] === right[6] &&
                left[8] === right[8] &&
                left[9] === right[9] &&
                left[10] === right[10] &&

                // Bottom row
                left[3] === right[3] &&
                left[7] === right[7] &&
                left[11] === right[11] &&
                left[15] === right[15]);
    };

    /**
     * Compares the provided matrices componentwise and returns
     * <code>true</code> if they are within the provided epsilon,
     * <code>false</code> otherwise.
     *
     * @param {Matrix4} [left] The first matrix.
     * @param {Matrix4} [right] The second matrix.
     * @param {Number} epsilon The epsilon to use for equality testing.
     * @returns {Boolean} <code>true</code> if left and right are within the provided epsilon, <code>false</code> otherwise.
     *
     * @example
     * //compares two Matrix4 instances
     *
     * // a = [10.5, 14.5, 18.5, 22.5]
     * //     [11.5, 15.5, 19.5, 23.5]
     * //     [12.5, 16.5, 20.5, 24.5]
     * //     [13.5, 17.5, 21.5, 25.5]
     *
     * // b = [10.0, 14.0, 18.0, 22.0]
     * //     [11.0, 15.0, 19.0, 23.0]
     * //     [12.0, 16.0, 20.0, 24.0]
     * //     [13.0, 17.0, 21.0, 25.0]
     *
     * if(Cesium.Matrix4.equalsEpsilon(a,b,0.1)){
     *      console.log("Difference between both the matrices is less than 0.1");
     * } else {
     *      console.log("Difference between both the matrices is not less than 0.1");
     * }
     *
     * //Prints "Difference between both the matrices is not less than 0.1" on the console
     */
    Matrix4.equalsEpsilon = function(left, right, epsilon) {
                Check.typeOf.number('epsilon', epsilon);
        
        return (left === right) ||
                (defined(left) &&
                defined(right) &&
                Math.abs(left[0] - right[0]) <= epsilon &&
                Math.abs(left[1] - right[1]) <= epsilon &&
                Math.abs(left[2] - right[2]) <= epsilon &&
                Math.abs(left[3] - right[3]) <= epsilon &&
                Math.abs(left[4] - right[4]) <= epsilon &&
                Math.abs(left[5] - right[5]) <= epsilon &&
                Math.abs(left[6] - right[6]) <= epsilon &&
                Math.abs(left[7] - right[7]) <= epsilon &&
                Math.abs(left[8] - right[8]) <= epsilon &&
                Math.abs(left[9] - right[9]) <= epsilon &&
                Math.abs(left[10] - right[10]) <= epsilon &&
                Math.abs(left[11] - right[11]) <= epsilon &&
                Math.abs(left[12] - right[12]) <= epsilon &&
                Math.abs(left[13] - right[13]) <= epsilon &&
                Math.abs(left[14] - right[14]) <= epsilon &&
                Math.abs(left[15] - right[15]) <= epsilon);
    };

    /**
     * Gets the translation portion of the provided matrix, assuming the matrix is a affine transformation matrix.
     *
     * @param {Matrix4} matrix The matrix to use.
     * @param {Cartesian3} result The object onto which to store the result.
     * @returns {Cartesian3} The modified result parameter.
     */
    Matrix4.getTranslation = function(matrix, result) {
                Check.typeOf.object('matrix', matrix);
        Check.typeOf.object('result', result);
        
        result.x = matrix[12];
        result.y = matrix[13];
        result.z = matrix[14];
        return result;
    };

    /**
     * Gets the upper left 3x3 rotation matrix of the provided matrix, assuming the matrix is a affine transformation matrix.
     *
     * @param {Matrix4} matrix The matrix to use.
     * @param {Matrix3} result The object onto which to store the result.
     * @returns {Matrix3} The modified result parameter.
     *
     * @example
     * // returns a Matrix3 instance from a Matrix4 instance
     *
     * // m = [10.0, 14.0, 18.0, 22.0]
     * //     [11.0, 15.0, 19.0, 23.0]
     * //     [12.0, 16.0, 20.0, 24.0]
     * //     [13.0, 17.0, 21.0, 25.0]
     *
     * var b = new Cesium.Matrix3();
     * Cesium.Matrix4.getRotation(m,b);
     *
     * // b = [10.0, 14.0, 18.0]
     * //     [11.0, 15.0, 19.0]
     * //     [12.0, 16.0, 20.0]
     */
    Matrix4.getRotation = function(matrix, result) {
                Check.typeOf.object('matrix', matrix);
        Check.typeOf.object('result', result);
        
        result[0] = matrix[0];
        result[1] = matrix[1];
        result[2] = matrix[2];
        result[3] = matrix[4];
        result[4] = matrix[5];
        result[5] = matrix[6];
        result[6] = matrix[8];
        result[7] = matrix[9];
        result[8] = matrix[10];
        return result;
    };

    var scratchInverseRotation = new Matrix3();
    var scratchMatrix3Zero = new Matrix3();
    var scratchBottomRow = new Cartesian4();
    var scratchExpectedBottomRow = new Cartesian4(0.0, 0.0, 0.0, 1.0);

     /**
      * Computes the inverse of the provided matrix using Cramers Rule.
      * If the determinant is zero, the matrix can not be inverted, and an exception is thrown.
      * If the matrix is an affine transformation matrix, it is more efficient
      * to invert it with {@link Matrix4.inverseTransformation}.
      *
      * @param {Matrix4} matrix The matrix to invert.
      * @param {Matrix4} result The object onto which to store the result.
      * @returns {Matrix4} The modified result parameter.
      *
      * @exception {RuntimeError} matrix is not invertible because its determinate is zero.
      */
    Matrix4.inverse = function(matrix, result) {
                Check.typeOf.object('matrix', matrix);
        Check.typeOf.object('result', result);
                //
        // Ported from:
        //   ftp://download.intel.com/design/PentiumIII/sml/24504301.pdf
        //
        var src0 = matrix[0];
        var src1 = matrix[4];
        var src2 = matrix[8];
        var src3 = matrix[12];
        var src4 = matrix[1];
        var src5 = matrix[5];
        var src6 = matrix[9];
        var src7 = matrix[13];
        var src8 = matrix[2];
        var src9 = matrix[6];
        var src10 = matrix[10];
        var src11 = matrix[14];
        var src12 = matrix[3];
        var src13 = matrix[7];
        var src14 = matrix[11];
        var src15 = matrix[15];

        // calculate pairs for first 8 elements (cofactors)
        var tmp0 = src10 * src15;
        var tmp1 = src11 * src14;
        var tmp2 = src9 * src15;
        var tmp3 = src11 * src13;
        var tmp4 = src9 * src14;
        var tmp5 = src10 * src13;
        var tmp6 = src8 * src15;
        var tmp7 = src11 * src12;
        var tmp8 = src8 * src14;
        var tmp9 = src10 * src12;
        var tmp10 = src8 * src13;
        var tmp11 = src9 * src12;

        // calculate first 8 elements (cofactors)
        var dst0 = (tmp0 * src5 + tmp3 * src6 + tmp4 * src7) - (tmp1 * src5 + tmp2 * src6 + tmp5 * src7);
        var dst1 = (tmp1 * src4 + tmp6 * src6 + tmp9 * src7) - (tmp0 * src4 + tmp7 * src6 + tmp8 * src7);
        var dst2 = (tmp2 * src4 + tmp7 * src5 + tmp10 * src7) - (tmp3 * src4 + tmp6 * src5 + tmp11 * src7);
        var dst3 = (tmp5 * src4 + tmp8 * src5 + tmp11 * src6) - (tmp4 * src4 + tmp9 * src5 + tmp10 * src6);
        var dst4 = (tmp1 * src1 + tmp2 * src2 + tmp5 * src3) - (tmp0 * src1 + tmp3 * src2 + tmp4 * src3);
        var dst5 = (tmp0 * src0 + tmp7 * src2 + tmp8 * src3) - (tmp1 * src0 + tmp6 * src2 + tmp9 * src3);
        var dst6 = (tmp3 * src0 + tmp6 * src1 + tmp11 * src3) - (tmp2 * src0 + tmp7 * src1 + tmp10 * src3);
        var dst7 = (tmp4 * src0 + tmp9 * src1 + tmp10 * src2) - (tmp5 * src0 + tmp8 * src1 + tmp11 * src2);

        // calculate pairs for second 8 elements (cofactors)
        tmp0 = src2 * src7;
        tmp1 = src3 * src6;
        tmp2 = src1 * src7;
        tmp3 = src3 * src5;
        tmp4 = src1 * src6;
        tmp5 = src2 * src5;
        tmp6 = src0 * src7;
        tmp7 = src3 * src4;
        tmp8 = src0 * src6;
        tmp9 = src2 * src4;
        tmp10 = src0 * src5;
        tmp11 = src1 * src4;

        // calculate second 8 elements (cofactors)
        var dst8 = (tmp0 * src13 + tmp3 * src14 + tmp4 * src15) - (tmp1 * src13 + tmp2 * src14 + tmp5 * src15);
        var dst9 = (tmp1 * src12 + tmp6 * src14 + tmp9 * src15) - (tmp0 * src12 + tmp7 * src14 + tmp8 * src15);
        var dst10 = (tmp2 * src12 + tmp7 * src13 + tmp10 * src15) - (tmp3 * src12 + tmp6 * src13 + tmp11 * src15);
        var dst11 = (tmp5 * src12 + tmp8 * src13 + tmp11 * src14) - (tmp4 * src12 + tmp9 * src13 + tmp10 * src14);
        var dst12 = (tmp2 * src10 + tmp5 * src11 + tmp1 * src9) - (tmp4 * src11 + tmp0 * src9 + tmp3 * src10);
        var dst13 = (tmp8 * src11 + tmp0 * src8 + tmp7 * src10) - (tmp6 * src10 + tmp9 * src11 + tmp1 * src8);
        var dst14 = (tmp6 * src9 + tmp11 * src11 + tmp3 * src8) - (tmp10 * src11 + tmp2 * src8 + tmp7 * src9);
        var dst15 = (tmp10 * src10 + tmp4 * src8 + tmp9 * src9) - (tmp8 * src9 + tmp11 * src10 + tmp5 * src8);

        // calculate determinant
        var det = src0 * dst0 + src1 * dst1 + src2 * dst2 + src3 * dst3;

        if (Math.abs(det) < CesiumMath.EPSILON21) {
                // Special case for a zero scale matrix that can occur, for example,
                // when a model's node has a [0, 0, 0] scale.
                if (Matrix3.equalsEpsilon(Matrix4.getRotation(matrix, scratchInverseRotation), scratchMatrix3Zero, CesiumMath.EPSILON7) &&
                Cartesian4.equals(Matrix4.getRow(matrix, 3, scratchBottomRow), scratchExpectedBottomRow)) {

                result[0] = 0.0;
                result[1] = 0.0;
                result[2] = 0.0;
                result[3] = 0.0;
                result[4] = 0.0;
                result[5] = 0.0;
                result[6] = 0.0;
                result[7] = 0.0;
                result[8] = 0.0;
                result[9] = 0.0;
                result[10] = 0.0;
                result[11] = 0.0;
                result[12] = -matrix[12];
                result[13] = -matrix[13];
                result[14] = -matrix[14];
                result[15] = 1.0;
                return result;
            }

            throw new RuntimeError('matrix is not invertible because its determinate is zero.');
        }

        // calculate matrix inverse
        det = 1.0 / det;

        result[0] = dst0 * det;
        result[1] = dst1 * det;
        result[2] = dst2 * det;
        result[3] = dst3 * det;
        result[4] = dst4 * det;
        result[5] = dst5 * det;
        result[6] = dst6 * det;
        result[7] = dst7 * det;
        result[8] = dst8 * det;
        result[9] = dst9 * det;
        result[10] = dst10 * det;
        result[11] = dst11 * det;
        result[12] = dst12 * det;
        result[13] = dst13 * det;
        result[14] = dst14 * det;
        result[15] = dst15 * det;
        return result;
    };

    /**
     * Computes the inverse of the provided matrix assuming it is
     * an affine transformation matrix, where the upper left 3x3 elements
     * are a rotation matrix, and the upper three elements in the fourth
     * column are the translation.  The bottom row is assumed to be [0, 0, 0, 1].
     * The matrix is not verified to be in the proper form.
     * This method is faster than computing the inverse for a general 4x4
     * matrix using {@link Matrix4.inverse}.
     *
     * @param {Matrix4} matrix The matrix to invert.
     * @param {Matrix4} result The object onto which to store the result.
     * @returns {Matrix4} The modified result parameter.
     */
    Matrix4.inverseTransformation = function(matrix, result) {
                Check.typeOf.object('matrix', matrix);
        Check.typeOf.object('result', result);
        
        //This function is an optimized version of the below 4 lines.
        //var rT = Matrix3.transpose(Matrix4.getRotation(matrix));
        //var rTN = Matrix3.negate(rT);
        //var rTT = Matrix3.multiplyByVector(rTN, Matrix4.getTranslation(matrix));
        //return Matrix4.fromRotationTranslation(rT, rTT, result);

        var matrix0 = matrix[0];
        var matrix1 = matrix[1];
        var matrix2 = matrix[2];
        var matrix4 = matrix[4];
        var matrix5 = matrix[5];
        var matrix6 = matrix[6];
        var matrix8 = matrix[8];
        var matrix9 = matrix[9];
        var matrix10 = matrix[10];

        var vX = matrix[12];
        var vY = matrix[13];
        var vZ = matrix[14];

        var x = -matrix0 * vX - matrix1 * vY - matrix2 * vZ;
        var y = -matrix4 * vX - matrix5 * vY - matrix6 * vZ;
        var z = -matrix8 * vX - matrix9 * vY - matrix10 * vZ;

        result[0] = matrix0;
        result[1] = matrix4;
        result[2] = matrix8;
        result[3] = 0.0;
        result[4] = matrix1;
        result[5] = matrix5;
        result[6] = matrix9;
        result[7] = 0.0;
        result[8] = matrix2;
        result[9] = matrix6;
        result[10] = matrix10;
        result[11] = 0.0;
        result[12] = x;
        result[13] = y;
        result[14] = z;
        result[15] = 1.0;
        return result;
    };

    /**
     * An immutable Matrix4 instance initialized to the identity matrix.
     *
     * @type {Matrix4}
     * @constant
     */
    Matrix4.IDENTITY = freezeObject(new Matrix4(1.0, 0.0, 0.0, 0.0,
                                                0.0, 1.0, 0.0, 0.0,
                                                0.0, 0.0, 1.0, 0.0,
                                                0.0, 0.0, 0.0, 1.0));

    /**
     * An immutable Matrix4 instance initialized to the zero matrix.
     *
     * @type {Matrix4}
     * @constant
     */
    Matrix4.ZERO = freezeObject(new Matrix4(0.0, 0.0, 0.0, 0.0,
                                            0.0, 0.0, 0.0, 0.0,
                                            0.0, 0.0, 0.0, 0.0,
                                            0.0, 0.0, 0.0, 0.0));

    /**
     * The index into Matrix4 for column 0, row 0.
     *
     * @type {Number}
     * @constant
     */
    Matrix4.COLUMN0ROW0 = 0;

    /**
     * The index into Matrix4 for column 0, row 1.
     *
     * @type {Number}
     * @constant
     */
    Matrix4.COLUMN0ROW1 = 1;

    /**
     * The index into Matrix4 for column 0, row 2.
     *
     * @type {Number}
     * @constant
     */
    Matrix4.COLUMN0ROW2 = 2;

    /**
     * The index into Matrix4 for column 0, row 3.
     *
     * @type {Number}
     * @constant
     */
    Matrix4.COLUMN0ROW3 = 3;

    /**
     * The index into Matrix4 for column 1, row 0.
     *
     * @type {Number}
     * @constant
     */
    Matrix4.COLUMN1ROW0 = 4;

    /**
     * The index into Matrix4 for column 1, row 1.
     *
     * @type {Number}
     * @constant
     */
    Matrix4.COLUMN1ROW1 = 5;

    /**
     * The index into Matrix4 for column 1, row 2.
     *
     * @type {Number}
     * @constant
     */
    Matrix4.COLUMN1ROW2 = 6;

    /**
     * The index into Matrix4 for column 1, row 3.
     *
     * @type {Number}
     * @constant
     */
    Matrix4.COLUMN1ROW3 = 7;

    /**
     * The index into Matrix4 for column 2, row 0.
     *
     * @type {Number}
     * @constant
     */
    Matrix4.COLUMN2ROW0 = 8;

    /**
     * The index into Matrix4 for column 2, row 1.
     *
     * @type {Number}
     * @constant
     */
    Matrix4.COLUMN2ROW1 = 9;

    /**
     * The index into Matrix4 for column 2, row 2.
     *
     * @type {Number}
     * @constant
     */
    Matrix4.COLUMN2ROW2 = 10;

    /**
     * The index into Matrix4 for column 2, row 3.
     *
     * @type {Number}
     * @constant
     */
    Matrix4.COLUMN2ROW3 = 11;

    /**
     * The index into Matrix4 for column 3, row 0.
     *
     * @type {Number}
     * @constant
     */
    Matrix4.COLUMN3ROW0 = 12;

    /**
     * The index into Matrix4 for column 3, row 1.
     *
     * @type {Number}
     * @constant
     */
    Matrix4.COLUMN3ROW1 = 13;

    /**
     * The index into Matrix4 for column 3, row 2.
     *
     * @type {Number}
     * @constant
     */
    Matrix4.COLUMN3ROW2 = 14;

    /**
     * The index into Matrix4 for column 3, row 3.
     *
     * @type {Number}
     * @constant
     */
    Matrix4.COLUMN3ROW3 = 15;

    defineProperties(Matrix4.prototype, {
        /**
         * Gets the number of items in the collection.
         * @memberof Matrix4.prototype
         *
         * @type {Number}
         */
        length : {
            get : function() {
                return Matrix4.packedLength;
            }
        }
    });

    /**
     * Duplicates the provided Matrix4 instance.
     *
     * @param {Matrix4} [result] The object onto which to store the result.
     * @returns {Matrix4} The modified result parameter or a new Matrix4 instance if one was not provided.
     */
    Matrix4.prototype.clone = function(result) {
        return Matrix4.clone(this, result);
    };

    /**
     * Compares this matrix to the provided matrix componentwise and returns
     * <code>true</code> if they are equal, <code>false</code> otherwise.
     *
     * @param {Matrix4} [right] The right hand side matrix.
     * @returns {Boolean} <code>true</code> if they are equal, <code>false</code> otherwise.
     */
    Matrix4.prototype.equals = function(right) {
        return Matrix4.equals(this, right);
    };

    /**
     * @private
     */
    Matrix4.equalsArray = function(matrix, array, offset) {
        return matrix[0] === array[offset] &&
               matrix[1] === array[offset + 1] &&
               matrix[2] === array[offset + 2] &&
               matrix[3] === array[offset + 3] &&
               matrix[4] === array[offset + 4] &&
               matrix[5] === array[offset + 5] &&
               matrix[6] === array[offset + 6] &&
               matrix[7] === array[offset + 7] &&
               matrix[8] === array[offset + 8] &&
               matrix[9] === array[offset + 9] &&
               matrix[10] === array[offset + 10] &&
               matrix[11] === array[offset + 11] &&
               matrix[12] === array[offset + 12] &&
               matrix[13] === array[offset + 13] &&
               matrix[14] === array[offset + 14] &&
               matrix[15] === array[offset + 15];
    };

    /**
     * Compares this matrix to the provided matrix componentwise and returns
     * <code>true</code> if they are within the provided epsilon,
     * <code>false</code> otherwise.
     *
     * @param {Matrix4} [right] The right hand side matrix.
     * @param {Number} epsilon The epsilon to use for equality testing.
     * @returns {Boolean} <code>true</code> if they are within the provided epsilon, <code>false</code> otherwise.
     */
    Matrix4.prototype.equalsEpsilon = function(right, epsilon) {
        return Matrix4.equalsEpsilon(this, right, epsilon);
    };

    /**
     * Computes a string representing this Matrix with each row being
     * on a separate line and in the format '(column0, column1, column2, column3)'.
     *
     * @returns {String} A string representing the provided Matrix with each row being on a separate line and in the format '(column0, column1, column2, column3)'.
     */
    Matrix4.prototype.toString = function() {
        return '(' + this[0] + ', ' + this[4] + ', ' + this[8] + ', ' + this[12] +')\n' +
               '(' + this[1] + ', ' + this[5] + ', ' + this[9] + ', ' + this[13] +')\n' +
               '(' + this[2] + ', ' + this[6] + ', ' + this[10] + ', ' + this[14] +')\n' +
               '(' + this[3] + ', ' + this[7] + ', ' + this[11] + ', ' + this[15] +')';
    };

    return Matrix4;
});

define('Core/Rectangle',[
        './Cartographic',
        './Check',
        './defaultValue',
        './defined',
        './defineProperties',
        './Ellipsoid',
        './freezeObject',
        './Math'
    ], function(
        Cartographic,
        Check,
        defaultValue,
        defined,
        defineProperties,
        Ellipsoid,
        freezeObject,
        CesiumMath) {
    'use strict';

    /**
     * A two dimensional region specified as longitude and latitude coordinates.
     *
     * @alias Rectangle
     * @constructor
     *
     * @param {Number} [west=0.0] The westernmost longitude, in radians, in the range [-Pi, Pi].
     * @param {Number} [south=0.0] The southernmost latitude, in radians, in the range [-Pi/2, Pi/2].
     * @param {Number} [east=0.0] The easternmost longitude, in radians, in the range [-Pi, Pi].
     * @param {Number} [north=0.0] The northernmost latitude, in radians, in the range [-Pi/2, Pi/2].
     *
     * @see Packable
     */
    function Rectangle(west, south, east, north) {
        /**
         * The westernmost longitude in radians in the range [-Pi, Pi].
         *
         * @type {Number}
         * @default 0.0
         */
        this.west = defaultValue(west, 0.0);

        /**
         * The southernmost latitude in radians in the range [-Pi/2, Pi/2].
         *
         * @type {Number}
         * @default 0.0
         */
        this.south = defaultValue(south, 0.0);

        /**
         * The easternmost longitude in radians in the range [-Pi, Pi].
         *
         * @type {Number}
         * @default 0.0
         */
        this.east = defaultValue(east, 0.0);

        /**
         * The northernmost latitude in radians in the range [-Pi/2, Pi/2].
         *
         * @type {Number}
         * @default 0.0
         */
        this.north = defaultValue(north, 0.0);
    }

    defineProperties(Rectangle.prototype, {
        /**
         * Gets the width of the rectangle in radians.
         * @memberof Rectangle.prototype
         * @type {Number}
         */
        width : {
            get : function() {
                return Rectangle.computeWidth(this);
            }
        },

        /**
         * Gets the height of the rectangle in radians.
         * @memberof Rectangle.prototype
         * @type {Number}
         */
        height : {
            get : function() {
                return Rectangle.computeHeight(this);
            }
        }
    });

    /**
     * The number of elements used to pack the object into an array.
     * @type {Number}
     */
    Rectangle.packedLength = 4;

    /**
     * Stores the provided instance into the provided array.
     *
     * @param {Rectangle} value The value to pack.
     * @param {Number[]} array The array to pack into.
     * @param {Number} [startingIndex=0] The index into the array at which to start packing the elements.
     *
     * @returns {Number[]} The array that was packed into
     */
    Rectangle.pack = function(value, array, startingIndex) {
                Check.typeOf.object('value', value);
        Check.defined('array', array);
        
        startingIndex = defaultValue(startingIndex, 0);

        array[startingIndex++] = value.west;
        array[startingIndex++] = value.south;
        array[startingIndex++] = value.east;
        array[startingIndex] = value.north;

        return array;
    };

    /**
     * Retrieves an instance from a packed array.
     *
     * @param {Number[]} array The packed array.
     * @param {Number} [startingIndex=0] The starting index of the element to be unpacked.
     * @param {Rectangle} [result] The object into which to store the result.
     * @returns {Rectangle} The modified result parameter or a new Rectangle instance if one was not provided.
     */
    Rectangle.unpack = function(array, startingIndex, result) {
                Check.defined('array', array);
        
        startingIndex = defaultValue(startingIndex, 0);

        if (!defined(result)) {
            result = new Rectangle();
        }

        result.west = array[startingIndex++];
        result.south = array[startingIndex++];
        result.east = array[startingIndex++];
        result.north = array[startingIndex];
        return result;
    };

    /**
     * Computes the width of a rectangle in radians.
     * @param {Rectangle} rectangle The rectangle to compute the width of.
     * @returns {Number} The width.
     */
    Rectangle.computeWidth = function(rectangle) {
                Check.typeOf.object('rectangle', rectangle);
                var east = rectangle.east;
        var west = rectangle.west;
        if (east < west) {
            east += CesiumMath.TWO_PI;
        }
        return east - west;
    };

    /**
     * Computes the height of a rectangle in radians.
     * @param {Rectangle} rectangle The rectangle to compute the height of.
     * @returns {Number} The height.
     */
    Rectangle.computeHeight = function(rectangle) {
                Check.typeOf.object('rectangle', rectangle);
                return rectangle.north - rectangle.south;
    };

    /**
     * Creates a rectangle given the boundary longitude and latitude in degrees.
     *
     * @param {Number} [west=0.0] The westernmost longitude in degrees in the range [-180.0, 180.0].
     * @param {Number} [south=0.0] The southernmost latitude in degrees in the range [-90.0, 90.0].
     * @param {Number} [east=0.0] The easternmost longitude in degrees in the range [-180.0, 180.0].
     * @param {Number} [north=0.0] The northernmost latitude in degrees in the range [-90.0, 90.0].
     * @param {Rectangle} [result] The object onto which to store the result, or undefined if a new instance should be created.
     * @returns {Rectangle} The modified result parameter or a new Rectangle instance if none was provided.
     *
     * @example
     * var rectangle = Cesium.Rectangle.fromDegrees(0.0, 20.0, 10.0, 30.0);
     */
    Rectangle.fromDegrees = function(west, south, east, north, result) {
        west = CesiumMath.toRadians(defaultValue(west, 0.0));
        south = CesiumMath.toRadians(defaultValue(south, 0.0));
        east = CesiumMath.toRadians(defaultValue(east, 0.0));
        north = CesiumMath.toRadians(defaultValue(north, 0.0));

        if (!defined(result)) {
            return new Rectangle(west, south, east, north);
        }

        result.west = west;
        result.south = south;
        result.east = east;
        result.north = north;

        return result;
    };

    /**
     * Creates a rectangle given the boundary longitude and latitude in radians.
     *
     * @param {Number} [west=0.0] The westernmost longitude in radians in the range [-Math.PI, Math.PI].
     * @param {Number} [south=0.0] The southernmost latitude in radians in the range [-Math.PI/2, Math.PI/2].
     * @param {Number} [east=0.0] The easternmost longitude in radians in the range [-Math.PI, Math.PI].
     * @param {Number} [north=0.0] The northernmost latitude in radians in the range [-Math.PI/2, Math.PI/2].
     * @param {Rectangle} [result] The object onto which to store the result, or undefined if a new instance should be created.
     * @returns {Rectangle} The modified result parameter or a new Rectangle instance if none was provided.
     *
     * @example
     * var rectangle = Cesium.Rectangle.fromRadians(0.0, Math.PI/4, Math.PI/8, 3*Math.PI/4);
     */
    Rectangle.fromRadians = function(west, south, east, north, result) {
        if (!defined(result)) {
            return new Rectangle(west, south, east, north);
        }

        result.west = defaultValue(west, 0.0);
        result.south = defaultValue(south, 0.0);
        result.east = defaultValue(east, 0.0);
        result.north = defaultValue(north, 0.0);

        return result;
    };

    /**
     * Creates the smallest possible Rectangle that encloses all positions in the provided array.
     *
     * @param {Cartographic[]} cartographics The list of Cartographic instances.
     * @param {Rectangle} [result] The object onto which to store the result, or undefined if a new instance should be created.
     * @returns {Rectangle} The modified result parameter or a new Rectangle instance if none was provided.
     */
    Rectangle.fromCartographicArray = function(cartographics, result) {
                Check.defined('cartographics', cartographics);
        
        var west = Number.MAX_VALUE;
        var east = -Number.MAX_VALUE;
        var westOverIDL = Number.MAX_VALUE;
        var eastOverIDL = -Number.MAX_VALUE;
        var south = Number.MAX_VALUE;
        var north = -Number.MAX_VALUE;

        for ( var i = 0, len = cartographics.length; i < len; i++) {
            var position = cartographics[i];
            west = Math.min(west, position.longitude);
            east = Math.max(east, position.longitude);
            south = Math.min(south, position.latitude);
            north = Math.max(north, position.latitude);

            var lonAdjusted = position.longitude >= 0 ?  position.longitude : position.longitude +  CesiumMath.TWO_PI;
            westOverIDL = Math.min(westOverIDL, lonAdjusted);
            eastOverIDL = Math.max(eastOverIDL, lonAdjusted);
        }

        if(east - west > eastOverIDL - westOverIDL) {
            west = westOverIDL;
            east = eastOverIDL;

            if (east > CesiumMath.PI) {
                east = east - CesiumMath.TWO_PI;
            }
            if (west > CesiumMath.PI) {
                west = west - CesiumMath.TWO_PI;
            }
        }

        if (!defined(result)) {
            return new Rectangle(west, south, east, north);
        }

        result.west = west;
        result.south = south;
        result.east = east;
        result.north = north;
        return result;
    };

    /**
     * Creates the smallest possible Rectangle that encloses all positions in the provided array.
     *
     * @param {Cartesian3[]} cartesians The list of Cartesian instances.
     * @param {Ellipsoid} [ellipsoid=Ellipsoid.WGS84] The ellipsoid the cartesians are on.
     * @param {Rectangle} [result] The object onto which to store the result, or undefined if a new instance should be created.
     * @returns {Rectangle} The modified result parameter or a new Rectangle instance if none was provided.
     */
    Rectangle.fromCartesianArray = function(cartesians, ellipsoid, result) {
                Check.defined('cartesians', cartesians);
                ellipsoid = defaultValue(ellipsoid, Ellipsoid.WGS84);

        var west = Number.MAX_VALUE;
        var east = -Number.MAX_VALUE;
        var westOverIDL = Number.MAX_VALUE;
        var eastOverIDL = -Number.MAX_VALUE;
        var south = Number.MAX_VALUE;
        var north = -Number.MAX_VALUE;

        for ( var i = 0, len = cartesians.length; i < len; i++) {
            var position = ellipsoid.cartesianToCartographic(cartesians[i]);
            west = Math.min(west, position.longitude);
            east = Math.max(east, position.longitude);
            south = Math.min(south, position.latitude);
            north = Math.max(north, position.latitude);

            var lonAdjusted = position.longitude >= 0 ?  position.longitude : position.longitude +  CesiumMath.TWO_PI;
            westOverIDL = Math.min(westOverIDL, lonAdjusted);
            eastOverIDL = Math.max(eastOverIDL, lonAdjusted);
        }

        if(east - west > eastOverIDL - westOverIDL) {
            west = westOverIDL;
            east = eastOverIDL;

            if (east > CesiumMath.PI) {
                east = east - CesiumMath.TWO_PI;
            }
            if (west > CesiumMath.PI) {
                west = west - CesiumMath.TWO_PI;
            }
        }

        if (!defined(result)) {
            return new Rectangle(west, south, east, north);
        }

        result.west = west;
        result.south = south;
        result.east = east;
        result.north = north;
        return result;
    };

    /**
     * Duplicates a Rectangle.
     *
     * @param {Rectangle} rectangle The rectangle to clone.
     * @param {Rectangle} [result] The object onto which to store the result, or undefined if a new instance should be created.
     * @returns {Rectangle} The modified result parameter or a new Rectangle instance if none was provided. (Returns undefined if rectangle is undefined)
     */
    Rectangle.clone = function(rectangle, result) {
        if (!defined(rectangle)) {
            return undefined;
        }

        if (!defined(result)) {
            return new Rectangle(rectangle.west, rectangle.south, rectangle.east, rectangle.north);
        }

        result.west = rectangle.west;
        result.south = rectangle.south;
        result.east = rectangle.east;
        result.north = rectangle.north;
        return result;
    };

    /**
     * Compares the provided Rectangles componentwise and returns
     * <code>true</code> if they pass an absolute or relative tolerance test,
     * <code>false</code> otherwise.
     *
     * @param {Rectangle} [left] The first Rectangle.
     * @param {Rectangle} [right] The second Rectangle.
     * @param {Number} absoluteEpsilon The absolute epsilon tolerance to use for equality testing.
     * @returns {Boolean} <code>true</code> if left and right are within the provided epsilon, <code>false</code> otherwise.
     */
    Rectangle.equalsEpsilon = function(left, right, absoluteEpsilon) {
                Check.typeOf.number('absoluteEpsilon', absoluteEpsilon);
        
        return (left === right) ||
               (defined(left) &&
                defined(right) &&
                (Math.abs(left.west - right.west) <= absoluteEpsilon) &&
                (Math.abs(left.south - right.south) <= absoluteEpsilon) &&
                (Math.abs(left.east - right.east) <= absoluteEpsilon) &&
                (Math.abs(left.north - right.north) <= absoluteEpsilon));
    };

    /**
     * Duplicates this Rectangle.
     *
     * @param {Rectangle} [result] The object onto which to store the result.
     * @returns {Rectangle} The modified result parameter or a new Rectangle instance if none was provided.
     */
    Rectangle.prototype.clone = function(result) {
        return Rectangle.clone(this, result);
    };

    /**
     * Compares the provided Rectangle with this Rectangle componentwise and returns
     * <code>true</code> if they are equal, <code>false</code> otherwise.
     *
     * @param {Rectangle} [other] The Rectangle to compare.
     * @returns {Boolean} <code>true</code> if the Rectangles are equal, <code>false</code> otherwise.
     */
    Rectangle.prototype.equals = function(other) {
        return Rectangle.equals(this, other);
    };

    /**
     * Compares the provided rectangles and returns <code>true</code> if they are equal,
     * <code>false</code> otherwise.
     *
     * @param {Rectangle} [left] The first Rectangle.
     * @param {Rectangle} [right] The second Rectangle.
     * @returns {Boolean} <code>true</code> if left and right are equal; otherwise <code>false</code>.
     */
    Rectangle.equals = function(left, right) {
        return (left === right) ||
               ((defined(left)) &&
                (defined(right)) &&
                (left.west === right.west) &&
                (left.south === right.south) &&
                (left.east === right.east) &&
                (left.north === right.north));
    };

    /**
     * Compares the provided Rectangle with this Rectangle componentwise and returns
     * <code>true</code> if they are within the provided epsilon,
     * <code>false</code> otherwise.
     *
     * @param {Rectangle} [other] The Rectangle to compare.
     * @param {Number} epsilon The epsilon to use for equality testing.
     * @returns {Boolean} <code>true</code> if the Rectangles are within the provided epsilon, <code>false</code> otherwise.
     */
    Rectangle.prototype.equalsEpsilon = function(other, epsilon) {
                Check.typeOf.number('epsilon', epsilon);
        
        return Rectangle.equalsEpsilon(this, other, epsilon);
    };

    /**
     * Checks a Rectangle's properties and throws if they are not in valid ranges.
     *
     * @param {Rectangle} rectangle The rectangle to validate
     *
     * @exception {DeveloperError} <code>north</code> must be in the interval [<code>-Pi/2</code>, <code>Pi/2</code>].
     * @exception {DeveloperError} <code>south</code> must be in the interval [<code>-Pi/2</code>, <code>Pi/2</code>].
     * @exception {DeveloperError} <code>east</code> must be in the interval [<code>-Pi</code>, <code>Pi</code>].
     * @exception {DeveloperError} <code>west</code> must be in the interval [<code>-Pi</code>, <code>Pi</code>].
     */
    Rectangle.validate = function(rectangle) {
                Check.typeOf.object('rectangle', rectangle);

        var north = rectangle.north;
        Check.typeOf.number.greaterThanOrEquals('north', north, -CesiumMath.PI_OVER_TWO);
        Check.typeOf.number.lessThanOrEquals('north', north, CesiumMath.PI_OVER_TWO);

        var south = rectangle.south;
        Check.typeOf.number.greaterThanOrEquals('south', south, -CesiumMath.PI_OVER_TWO);
        Check.typeOf.number.lessThanOrEquals('south', south, CesiumMath.PI_OVER_TWO);

        var west = rectangle.west;
        Check.typeOf.number.greaterThanOrEquals('west', west, -Math.PI);
        Check.typeOf.number.lessThanOrEquals('west', west, Math.PI);

        var east = rectangle.east;
        Check.typeOf.number.greaterThanOrEquals('east', east, -Math.PI);
        Check.typeOf.number.lessThanOrEquals('east', east, Math.PI);
            };

    /**
     * Computes the southwest corner of a rectangle.
     *
     * @param {Rectangle} rectangle The rectangle for which to find the corner
     * @param {Cartographic} [result] The object onto which to store the result.
     * @returns {Cartographic} The modified result parameter or a new Cartographic instance if none was provided.
     */
    Rectangle.southwest = function(rectangle, result) {
                Check.typeOf.object('rectangle', rectangle);
        
        if (!defined(result)) {
            return new Cartographic(rectangle.west, rectangle.south);
        }
        result.longitude = rectangle.west;
        result.latitude = rectangle.south;
        result.height = 0.0;
        return result;
    };

    /**
     * Computes the northwest corner of a rectangle.
     *
     * @param {Rectangle} rectangle The rectangle for which to find the corner
     * @param {Cartographic} [result] The object onto which to store the result.
     * @returns {Cartographic} The modified result parameter or a new Cartographic instance if none was provided.
     */
    Rectangle.northwest = function(rectangle, result) {
                Check.typeOf.object('rectangle', rectangle);
        
        if (!defined(result)) {
            return new Cartographic(rectangle.west, rectangle.north);
        }
        result.longitude = rectangle.west;
        result.latitude = rectangle.north;
        result.height = 0.0;
        return result;
    };

    /**
     * Computes the northeast corner of a rectangle.
     *
     * @param {Rectangle} rectangle The rectangle for which to find the corner
     * @param {Cartographic} [result] The object onto which to store the result.
     * @returns {Cartographic} The modified result parameter or a new Cartographic instance if none was provided.
     */
    Rectangle.northeast = function(rectangle, result) {
                Check.typeOf.object('rectangle', rectangle);
        
        if (!defined(result)) {
            return new Cartographic(rectangle.east, rectangle.north);
        }
        result.longitude = rectangle.east;
        result.latitude = rectangle.north;
        result.height = 0.0;
        return result;
    };

    /**
     * Computes the southeast corner of a rectangle.
     *
     * @param {Rectangle} rectangle The rectangle for which to find the corner
     * @param {Cartographic} [result] The object onto which to store the result.
     * @returns {Cartographic} The modified result parameter or a new Cartographic instance if none was provided.
     */
    Rectangle.southeast = function(rectangle, result) {
                Check.typeOf.object('rectangle', rectangle);
        
        if (!defined(result)) {
            return new Cartographic(rectangle.east, rectangle.south);
        }
        result.longitude = rectangle.east;
        result.latitude = rectangle.south;
        result.height = 0.0;
        return result;
    };

    /**
     * Computes the center of a rectangle.
     *
     * @param {Rectangle} rectangle The rectangle for which to find the center
     * @param {Cartographic} [result] The object onto which to store the result.
     * @returns {Cartographic} The modified result parameter or a new Cartographic instance if none was provided.
     */
    Rectangle.center = function(rectangle, result) {
                Check.typeOf.object('rectangle', rectangle);
        
        var east = rectangle.east;
        var west = rectangle.west;

        if (east < west) {
            east += CesiumMath.TWO_PI;
        }

        var longitude = CesiumMath.negativePiToPi((west + east) * 0.5);
        var latitude = (rectangle.south + rectangle.north) * 0.5;

        if (!defined(result)) {
            return new Cartographic(longitude, latitude);
        }

        result.longitude = longitude;
        result.latitude = latitude;
        result.height = 0.0;
        return result;
    };

    /**
     * Computes the intersection of two rectangles.  This function assumes that the rectangle's coordinates are
     * latitude and longitude in radians and produces a correct intersection, taking into account the fact that
     * the same angle can be represented with multiple values as well as the wrapping of longitude at the
     * anti-meridian.  For a simple intersection that ignores these factors and can be used with projected
     * coordinates, see {@link Rectangle.simpleIntersection}.
     *
     * @param {Rectangle} rectangle On rectangle to find an intersection
     * @param {Rectangle} otherRectangle Another rectangle to find an intersection
     * @param {Rectangle} [result] The object onto which to store the result.
     * @returns {Rectangle|undefined} The modified result parameter, a new Rectangle instance if none was provided or undefined if there is no intersection.
     */
    Rectangle.intersection = function(rectangle, otherRectangle, result) {
                Check.typeOf.object('rectangle', rectangle);
        Check.typeOf.object('otherRectangle', otherRectangle);
        
        var rectangleEast = rectangle.east;
        var rectangleWest = rectangle.west;

        var otherRectangleEast = otherRectangle.east;
        var otherRectangleWest = otherRectangle.west;

        if (rectangleEast < rectangleWest && otherRectangleEast > 0.0) {
            rectangleEast += CesiumMath.TWO_PI;
        } else if (otherRectangleEast < otherRectangleWest && rectangleEast > 0.0) {
            otherRectangleEast += CesiumMath.TWO_PI;
        }

        if (rectangleEast < rectangleWest && otherRectangleWest < 0.0) {
            otherRectangleWest += CesiumMath.TWO_PI;
        } else if (otherRectangleEast < otherRectangleWest && rectangleWest < 0.0) {
            rectangleWest += CesiumMath.TWO_PI;
        }

        var west = CesiumMath.negativePiToPi(Math.max(rectangleWest, otherRectangleWest));
        var east = CesiumMath.negativePiToPi(Math.min(rectangleEast, otherRectangleEast));

        if ((rectangle.west < rectangle.east || otherRectangle.west < otherRectangle.east) && east <= west) {
            return undefined;
        }

        var south = Math.max(rectangle.south, otherRectangle.south);
        var north = Math.min(rectangle.north, otherRectangle.north);

        if (south >= north) {
            return undefined;
        }

        if (!defined(result)) {
            return new Rectangle(west, south, east, north);
        }
        result.west = west;
        result.south = south;
        result.east = east;
        result.north = north;
        return result;
    };

    /**
     * Computes a simple intersection of two rectangles.  Unlike {@link Rectangle.intersection}, this function
     * does not attempt to put the angular coordinates into a consistent range or to account for crossing the
     * anti-meridian.  As such, it can be used for rectangles where the coordinates are not simply latitude
     * and longitude (i.e. projected coordinates).
     *
     * @param {Rectangle} rectangle On rectangle to find an intersection
     * @param {Rectangle} otherRectangle Another rectangle to find an intersection
     * @param {Rectangle} [result] The object onto which to store the result.
     * @returns {Rectangle|undefined} The modified result parameter, a new Rectangle instance if none was provided or undefined if there is no intersection.
     */
    Rectangle.simpleIntersection = function(rectangle, otherRectangle, result) {
                Check.typeOf.object('rectangle', rectangle);
        Check.typeOf.object('otherRectangle', otherRectangle);
        
        var west = Math.max(rectangle.west, otherRectangle.west);
        var south = Math.max(rectangle.south, otherRectangle.south);
        var east = Math.min(rectangle.east, otherRectangle.east);
        var north = Math.min(rectangle.north, otherRectangle.north);

        if (south >= north || west >= east) {
            return undefined;
        }

        if (!defined(result)) {
            return new Rectangle(west, south, east, north);
        }

        result.west = west;
        result.south = south;
        result.east = east;
        result.north = north;
        return result;
    };

    /**
     * Computes a rectangle that is the union of two rectangles.
     *
     * @param {Rectangle} rectangle A rectangle to enclose in rectangle.
     * @param {Rectangle} otherRectangle A rectangle to enclose in a rectangle.
     * @param {Rectangle} [result] The object onto which to store the result.
     * @returns {Rectangle} The modified result parameter or a new Rectangle instance if none was provided.
     */
    Rectangle.union = function(rectangle, otherRectangle, result) {
                Check.typeOf.object('rectangle', rectangle);
        Check.typeOf.object('otherRectangle', otherRectangle);
        
        if (!defined(result)) {
            result = new Rectangle();
        }

        var rectangleEast = rectangle.east;
        var rectangleWest = rectangle.west;

        var otherRectangleEast = otherRectangle.east;
        var otherRectangleWest = otherRectangle.west;

        if (rectangleEast < rectangleWest && otherRectangleEast > 0.0) {
            rectangleEast += CesiumMath.TWO_PI;
        } else if (otherRectangleEast < otherRectangleWest && rectangleEast > 0.0) {
            otherRectangleEast += CesiumMath.TWO_PI;
        }

        if (rectangleEast < rectangleWest && otherRectangleWest < 0.0) {
            otherRectangleWest += CesiumMath.TWO_PI;
        } else if (otherRectangleEast < otherRectangleWest && rectangleWest < 0.0) {
            rectangleWest += CesiumMath.TWO_PI;
        }

        var west = CesiumMath.convertLongitudeRange(Math.min(rectangleWest, otherRectangleWest));
        var east = CesiumMath.convertLongitudeRange(Math.max(rectangleEast, otherRectangleEast));

        result.west = west;
        result.south = Math.min(rectangle.south, otherRectangle.south);
        result.east = east;
        result.north = Math.max(rectangle.north, otherRectangle.north);

        return result;
    };

    /**
     * Computes a rectangle by enlarging the provided rectangle until it contains the provided cartographic.
     *
     * @param {Rectangle} rectangle A rectangle to expand.
     * @param {Cartographic} cartographic A cartographic to enclose in a rectangle.
     * @param {Rectangle} [result] The object onto which to store the result.
     * @returns {Rectangle} The modified result parameter or a new Rectangle instance if one was not provided.
     */
    Rectangle.expand = function(rectangle, cartographic, result) {
                Check.typeOf.object('rectangle', rectangle);
        Check.typeOf.object('cartographic', cartographic);
        
        if (!defined(result)) {
            result = new Rectangle();
        }

        result.west = Math.min(rectangle.west, cartographic.longitude);
        result.south = Math.min(rectangle.south, cartographic.latitude);
        result.east = Math.max(rectangle.east, cartographic.longitude);
        result.north = Math.max(rectangle.north, cartographic.latitude);

        return result;
    };

    /**
     * Returns true if the cartographic is on or inside the rectangle, false otherwise.
     *
     * @param {Rectangle} rectangle The rectangle
     * @param {Cartographic} cartographic The cartographic to test.
     * @returns {Boolean} true if the provided cartographic is inside the rectangle, false otherwise.
     */
    Rectangle.contains = function(rectangle, cartographic) {
                Check.typeOf.object('rectangle', rectangle);
        Check.typeOf.object('cartographic', cartographic);
        
        var longitude = cartographic.longitude;
        var latitude = cartographic.latitude;

        var west = rectangle.west;
        var east = rectangle.east;

        if (east < west) {
            east += CesiumMath.TWO_PI;
            if (longitude < 0.0) {
                longitude += CesiumMath.TWO_PI;
            }
        }
        return (longitude > west || CesiumMath.equalsEpsilon(longitude, west, CesiumMath.EPSILON14)) &&
               (longitude < east || CesiumMath.equalsEpsilon(longitude, east, CesiumMath.EPSILON14)) &&
               latitude >= rectangle.south &&
               latitude <= rectangle.north;
    };

    var subsampleLlaScratch = new Cartographic();
    /**
     * Samples a rectangle so that it includes a list of Cartesian points suitable for passing to
     * {@link BoundingSphere#fromPoints}.  Sampling is necessary to account
     * for rectangles that cover the poles or cross the equator.
     *
     * @param {Rectangle} rectangle The rectangle to subsample.
     * @param {Ellipsoid} [ellipsoid=Ellipsoid.WGS84] The ellipsoid to use.
     * @param {Number} [surfaceHeight=0.0] The height of the rectangle above the ellipsoid.
     * @param {Cartesian3[]} [result] The array of Cartesians onto which to store the result.
     * @returns {Cartesian3[]} The modified result parameter or a new Array of Cartesians instances if none was provided.
     */
    Rectangle.subsample = function(rectangle, ellipsoid, surfaceHeight, result) {
                Check.typeOf.object('rectangle', rectangle);
        
        ellipsoid = defaultValue(ellipsoid, Ellipsoid.WGS84);
        surfaceHeight = defaultValue(surfaceHeight, 0.0);

        if (!defined(result)) {
            result = [];
        }
        var length = 0;

        var north = rectangle.north;
        var south = rectangle.south;
        var east = rectangle.east;
        var west = rectangle.west;

        var lla = subsampleLlaScratch;
        lla.height = surfaceHeight;

        lla.longitude = west;
        lla.latitude = north;
        result[length] = ellipsoid.cartographicToCartesian(lla, result[length]);
        length++;

        lla.longitude = east;
        result[length] = ellipsoid.cartographicToCartesian(lla, result[length]);
        length++;

        lla.latitude = south;
        result[length] = ellipsoid.cartographicToCartesian(lla, result[length]);
        length++;

        lla.longitude = west;
        result[length] = ellipsoid.cartographicToCartesian(lla, result[length]);
        length++;

        if (north < 0.0) {
            lla.latitude = north;
        } else if (south > 0.0) {
            lla.latitude = south;
        } else {
            lla.latitude = 0.0;
        }

        for ( var i = 1; i < 8; ++i) {
            lla.longitude = -Math.PI + i * CesiumMath.PI_OVER_TWO;
            if (Rectangle.contains(rectangle, lla)) {
                result[length] = ellipsoid.cartographicToCartesian(lla, result[length]);
                length++;
            }
        }

        if (lla.latitude === 0.0) {
            lla.longitude = west;
            result[length] = ellipsoid.cartographicToCartesian(lla, result[length]);
            length++;
            lla.longitude = east;
            result[length] = ellipsoid.cartographicToCartesian(lla, result[length]);
            length++;
        }
        result.length = length;
        return result;
    };

    /**
     * The largest possible rectangle.
     *
     * @type {Rectangle}
     * @constant
    */
    Rectangle.MAX_VALUE = freezeObject(new Rectangle(-Math.PI, -CesiumMath.PI_OVER_TWO, Math.PI, CesiumMath.PI_OVER_TWO));

    return Rectangle;
});

define('Core/BoundingSphere',[
        './Cartesian3',
        './Cartographic',
        './Check',
        './defaultValue',
        './defined',
        './Ellipsoid',
        './GeographicProjection',
        './Intersect',
        './Interval',
        './Math',
        './Matrix3',
        './Matrix4',
        './Rectangle'
    ], function(
        Cartesian3,
        Cartographic,
        Check,
        defaultValue,
        defined,
        Ellipsoid,
        GeographicProjection,
        Intersect,
        Interval,
        CesiumMath,
        Matrix3,
        Matrix4,
        Rectangle) {
    'use strict';

    /**
     * A bounding sphere with a center and a radius.
     * @alias BoundingSphere
     * @constructor
     *
     * @param {Cartesian3} [center=Cartesian3.ZERO] The center of the bounding sphere.
     * @param {Number} [radius=0.0] The radius of the bounding sphere.
     *
     * @see AxisAlignedBoundingBox
     * @see BoundingRectangle
     * @see Packable
     */
    function BoundingSphere(center, radius) {
        /**
         * The center point of the sphere.
         * @type {Cartesian3}
         * @default {@link Cartesian3.ZERO}
         */
        this.center = Cartesian3.clone(defaultValue(center, Cartesian3.ZERO));

        /**
         * The radius of the sphere.
         * @type {Number}
         * @default 0.0
         */
        this.radius = defaultValue(radius, 0.0);
    }

    var fromPointsXMin = new Cartesian3();
    var fromPointsYMin = new Cartesian3();
    var fromPointsZMin = new Cartesian3();
    var fromPointsXMax = new Cartesian3();
    var fromPointsYMax = new Cartesian3();
    var fromPointsZMax = new Cartesian3();
    var fromPointsCurrentPos = new Cartesian3();
    var fromPointsScratch = new Cartesian3();
    var fromPointsRitterCenter = new Cartesian3();
    var fromPointsMinBoxPt = new Cartesian3();
    var fromPointsMaxBoxPt = new Cartesian3();
    var fromPointsNaiveCenterScratch = new Cartesian3();
    var volumeConstant = (4.0 / 3.0) * CesiumMath.PI;

    /**
     * Computes a tight-fitting bounding sphere enclosing a list of 3D Cartesian points.
     * The bounding sphere is computed by running two algorithms, a naive algorithm and
     * Ritter's algorithm. The smaller of the two spheres is used to ensure a tight fit.
     *
     * @param {Cartesian3[]} [positions] An array of points that the bounding sphere will enclose.  Each point must have <code>x</code>, <code>y</code>, and <code>z</code> properties.
     * @param {BoundingSphere} [result] The object onto which to store the result.
     * @returns {BoundingSphere} The modified result parameter or a new BoundingSphere instance if one was not provided.
     *
     * @see {@link http://blogs.agi.com/insight3d/index.php/2008/02/04/a-bounding/|Bounding Sphere computation article}
     */
    BoundingSphere.fromPoints = function(positions, result) {
        if (!defined(result)) {
            result = new BoundingSphere();
        }

        if (!defined(positions) || positions.length === 0) {
            result.center = Cartesian3.clone(Cartesian3.ZERO, result.center);
            result.radius = 0.0;
            return result;
        }

        var currentPos = Cartesian3.clone(positions[0], fromPointsCurrentPos);

        var xMin = Cartesian3.clone(currentPos, fromPointsXMin);
        var yMin = Cartesian3.clone(currentPos, fromPointsYMin);
        var zMin = Cartesian3.clone(currentPos, fromPointsZMin);

        var xMax = Cartesian3.clone(currentPos, fromPointsXMax);
        var yMax = Cartesian3.clone(currentPos, fromPointsYMax);
        var zMax = Cartesian3.clone(currentPos, fromPointsZMax);

        var numPositions = positions.length;
        var i;
        for (i = 1; i < numPositions; i++) {
            Cartesian3.clone(positions[i], currentPos);

            var x = currentPos.x;
            var y = currentPos.y;
            var z = currentPos.z;

            // Store points containing the the smallest and largest components
            if (x < xMin.x) {
                Cartesian3.clone(currentPos, xMin);
            }

            if (x > xMax.x) {
                Cartesian3.clone(currentPos, xMax);
            }

            if (y < yMin.y) {
                Cartesian3.clone(currentPos, yMin);
            }

            if (y > yMax.y) {
                Cartesian3.clone(currentPos, yMax);
            }

            if (z < zMin.z) {
                Cartesian3.clone(currentPos, zMin);
            }

            if (z > zMax.z) {
                Cartesian3.clone(currentPos, zMax);
            }
        }

        // Compute x-, y-, and z-spans (Squared distances b/n each component's min. and max.).
        var xSpan = Cartesian3.magnitudeSquared(Cartesian3.subtract(xMax, xMin, fromPointsScratch));
        var ySpan = Cartesian3.magnitudeSquared(Cartesian3.subtract(yMax, yMin, fromPointsScratch));
        var zSpan = Cartesian3.magnitudeSquared(Cartesian3.subtract(zMax, zMin, fromPointsScratch));

        // Set the diameter endpoints to the largest span.
        var diameter1 = xMin;
        var diameter2 = xMax;
        var maxSpan = xSpan;
        if (ySpan > maxSpan) {
            maxSpan = ySpan;
            diameter1 = yMin;
            diameter2 = yMax;
        }
        if (zSpan > maxSpan) {
            maxSpan = zSpan;
            diameter1 = zMin;
            diameter2 = zMax;
        }

        // Calculate the center of the initial sphere found by Ritter's algorithm
        var ritterCenter = fromPointsRitterCenter;
        ritterCenter.x = (diameter1.x + diameter2.x) * 0.5;
        ritterCenter.y = (diameter1.y + diameter2.y) * 0.5;
        ritterCenter.z = (diameter1.z + diameter2.z) * 0.5;

        // Calculate the radius of the initial sphere found by Ritter's algorithm
        var radiusSquared = Cartesian3.magnitudeSquared(Cartesian3.subtract(diameter2, ritterCenter, fromPointsScratch));
        var ritterRadius = Math.sqrt(radiusSquared);

        // Find the center of the sphere found using the Naive method.
        var minBoxPt = fromPointsMinBoxPt;
        minBoxPt.x = xMin.x;
        minBoxPt.y = yMin.y;
        minBoxPt.z = zMin.z;

        var maxBoxPt = fromPointsMaxBoxPt;
        maxBoxPt.x = xMax.x;
        maxBoxPt.y = yMax.y;
        maxBoxPt.z = zMax.z;

        var naiveCenter = Cartesian3.midpoint(minBoxPt, maxBoxPt, fromPointsNaiveCenterScratch);

        // Begin 2nd pass to find naive radius and modify the ritter sphere.
        var naiveRadius = 0;
        for (i = 0; i < numPositions; i++) {
            Cartesian3.clone(positions[i], currentPos);

            // Find the furthest point from the naive center to calculate the naive radius.
            var r = Cartesian3.magnitude(Cartesian3.subtract(currentPos, naiveCenter, fromPointsScratch));
            if (r > naiveRadius) {
                naiveRadius = r;
            }

            // Make adjustments to the Ritter Sphere to include all points.
            var oldCenterToPointSquared = Cartesian3.magnitudeSquared(Cartesian3.subtract(currentPos, ritterCenter, fromPointsScratch));
            if (oldCenterToPointSquared > radiusSquared) {
                var oldCenterToPoint = Math.sqrt(oldCenterToPointSquared);
                // Calculate new radius to include the point that lies outside
                ritterRadius = (ritterRadius + oldCenterToPoint) * 0.5;
                radiusSquared = ritterRadius * ritterRadius;
                // Calculate center of new Ritter sphere
                var oldToNew = oldCenterToPoint - ritterRadius;
                ritterCenter.x = (ritterRadius * ritterCenter.x + oldToNew * currentPos.x) / oldCenterToPoint;
                ritterCenter.y = (ritterRadius * ritterCenter.y + oldToNew * currentPos.y) / oldCenterToPoint;
                ritterCenter.z = (ritterRadius * ritterCenter.z + oldToNew * currentPos.z) / oldCenterToPoint;
            }
        }

        if (ritterRadius < naiveRadius) {
            Cartesian3.clone(ritterCenter, result.center);
            result.radius = ritterRadius;
        } else {
            Cartesian3.clone(naiveCenter, result.center);
            result.radius = naiveRadius;
        }

        return result;
    };

    var defaultProjection = new GeographicProjection();
    var fromRectangle2DLowerLeft = new Cartesian3();
    var fromRectangle2DUpperRight = new Cartesian3();
    var fromRectangle2DSouthwest = new Cartographic();
    var fromRectangle2DNortheast = new Cartographic();

    /**
     * Computes a bounding sphere from a rectangle projected in 2D.
     *
     * @param {Rectangle} [rectangle] The rectangle around which to create a bounding sphere.
     * @param {Object} [projection=GeographicProjection] The projection used to project the rectangle into 2D.
     * @param {BoundingSphere} [result] The object onto which to store the result.
     * @returns {BoundingSphere} The modified result parameter or a new BoundingSphere instance if none was provided.
     */
    BoundingSphere.fromRectangle2D = function(rectangle, projection, result) {
        return BoundingSphere.fromRectangleWithHeights2D(rectangle, projection, 0.0, 0.0, result);
    };

    /**
     * Computes a bounding sphere from a rectangle projected in 2D.  The bounding sphere accounts for the
     * object's minimum and maximum heights over the rectangle.
     *
     * @param {Rectangle} [rectangle] The rectangle around which to create a bounding sphere.
     * @param {Object} [projection=GeographicProjection] The projection used to project the rectangle into 2D.
     * @param {Number} [minimumHeight=0.0] The minimum height over the rectangle.
     * @param {Number} [maximumHeight=0.0] The maximum height over the rectangle.
     * @param {BoundingSphere} [result] The object onto which to store the result.
     * @returns {BoundingSphere} The modified result parameter or a new BoundingSphere instance if none was provided.
     */
    BoundingSphere.fromRectangleWithHeights2D = function(rectangle, projection, minimumHeight, maximumHeight, result) {
        if (!defined(result)) {
            result = new BoundingSphere();
        }

        if (!defined(rectangle)) {
            result.center = Cartesian3.clone(Cartesian3.ZERO, result.center);
            result.radius = 0.0;
            return result;
        }

        projection = defaultValue(projection, defaultProjection);

        Rectangle.southwest(rectangle, fromRectangle2DSouthwest);
        fromRectangle2DSouthwest.height = minimumHeight;
        Rectangle.northeast(rectangle, fromRectangle2DNortheast);
        fromRectangle2DNortheast.height = maximumHeight;

        var lowerLeft = projection.project(fromRectangle2DSouthwest, fromRectangle2DLowerLeft);
        var upperRight = projection.project(fromRectangle2DNortheast, fromRectangle2DUpperRight);

        var width = upperRight.x - lowerLeft.x;
        var height = upperRight.y - lowerLeft.y;
        var elevation = upperRight.z - lowerLeft.z;

        result.radius = Math.sqrt(width * width + height * height + elevation * elevation) * 0.5;
        var center = result.center;
        center.x = lowerLeft.x + width * 0.5;
        center.y = lowerLeft.y + height * 0.5;
        center.z = lowerLeft.z + elevation * 0.5;
        return result;
    };

    var fromRectangle3DScratch = [];

    /**
     * Computes a bounding sphere from a rectangle in 3D. The bounding sphere is created using a subsample of points
     * on the ellipsoid and contained in the rectangle. It may not be accurate for all rectangles on all types of ellipsoids.
     *
     * @param {Rectangle} [rectangle] The valid rectangle used to create a bounding sphere.
     * @param {Ellipsoid} [ellipsoid=Ellipsoid.WGS84] The ellipsoid used to determine positions of the rectangle.
     * @param {Number} [surfaceHeight=0.0] The height above the surface of the ellipsoid.
     * @param {BoundingSphere} [result] The object onto which to store the result.
     * @returns {BoundingSphere} The modified result parameter or a new BoundingSphere instance if none was provided.
     */
    BoundingSphere.fromRectangle3D = function(rectangle, ellipsoid, surfaceHeight, result) {
        ellipsoid = defaultValue(ellipsoid, Ellipsoid.WGS84);
        surfaceHeight = defaultValue(surfaceHeight, 0.0);

        if (!defined(result)) {
            result = new BoundingSphere();
        }

        if (!defined(rectangle)) {
            result.center = Cartesian3.clone(Cartesian3.ZERO, result.center);
            result.radius = 0.0;
            return result;
        }

        var positions = Rectangle.subsample(rectangle, ellipsoid, surfaceHeight, fromRectangle3DScratch);
        return BoundingSphere.fromPoints(positions, result);
    };

    /**
     * Computes a tight-fitting bounding sphere enclosing a list of 3D points, where the points are
     * stored in a flat array in X, Y, Z, order.  The bounding sphere is computed by running two
     * algorithms, a naive algorithm and Ritter's algorithm. The smaller of the two spheres is used to
     * ensure a tight fit.
     *
     * @param {Number[]} [positions] An array of points that the bounding sphere will enclose.  Each point
     *        is formed from three elements in the array in the order X, Y, Z.
     * @param {Cartesian3} [center=Cartesian3.ZERO] The position to which the positions are relative, which need not be the
     *        origin of the coordinate system.  This is useful when the positions are to be used for
     *        relative-to-center (RTC) rendering.
     * @param {Number} [stride=3] The number of array elements per vertex.  It must be at least 3, but it may
     *        be higher.  Regardless of the value of this parameter, the X coordinate of the first position
     *        is at array index 0, the Y coordinate is at array index 1, and the Z coordinate is at array index
     *        2.  When stride is 3, the X coordinate of the next position then begins at array index 3.  If
     *        the stride is 5, however, two array elements are skipped and the next position begins at array
     *        index 5.
     * @param {BoundingSphere} [result] The object onto which to store the result.
     * @returns {BoundingSphere} The modified result parameter or a new BoundingSphere instance if one was not provided.
     *
     * @example
     * // Compute the bounding sphere from 3 positions, each specified relative to a center.
     * // In addition to the X, Y, and Z coordinates, the points array contains two additional
     * // elements per point which are ignored for the purpose of computing the bounding sphere.
     * var center = new Cesium.Cartesian3(1.0, 2.0, 3.0);
     * var points = [1.0, 2.0, 3.0, 0.1, 0.2,
     *               4.0, 5.0, 6.0, 0.1, 0.2,
     *               7.0, 8.0, 9.0, 0.1, 0.2];
     * var sphere = Cesium.BoundingSphere.fromVertices(points, center, 5);
     *
     * @see {@link http://blogs.agi.com/insight3d/index.php/2008/02/04/a-bounding/|Bounding Sphere computation article}
     */
    BoundingSphere.fromVertices = function(positions, center, stride, result) {
        if (!defined(result)) {
            result = new BoundingSphere();
        }

        if (!defined(positions) || positions.length === 0) {
            result.center = Cartesian3.clone(Cartesian3.ZERO, result.center);
            result.radius = 0.0;
            return result;
        }

        center = defaultValue(center, Cartesian3.ZERO);

        stride = defaultValue(stride, 3);

                Check.typeOf.number.greaterThanOrEquals('stride', stride, 3);
        
        var currentPos = fromPointsCurrentPos;
        currentPos.x = positions[0] + center.x;
        currentPos.y = positions[1] + center.y;
        currentPos.z = positions[2] + center.z;

        var xMin = Cartesian3.clone(currentPos, fromPointsXMin);
        var yMin = Cartesian3.clone(currentPos, fromPointsYMin);
        var zMin = Cartesian3.clone(currentPos, fromPointsZMin);

        var xMax = Cartesian3.clone(currentPos, fromPointsXMax);
        var yMax = Cartesian3.clone(currentPos, fromPointsYMax);
        var zMax = Cartesian3.clone(currentPos, fromPointsZMax);

        var numElements = positions.length;
        var i;
        for (i = 0; i < numElements; i += stride) {
            var x = positions[i] + center.x;
            var y = positions[i + 1] + center.y;
            var z = positions[i + 2] + center.z;

            currentPos.x = x;
            currentPos.y = y;
            currentPos.z = z;

            // Store points containing the the smallest and largest components
            if (x < xMin.x) {
                Cartesian3.clone(currentPos, xMin);
            }

            if (x > xMax.x) {
                Cartesian3.clone(currentPos, xMax);
            }

            if (y < yMin.y) {
                Cartesian3.clone(currentPos, yMin);
            }

            if (y > yMax.y) {
                Cartesian3.clone(currentPos, yMax);
            }

            if (z < zMin.z) {
                Cartesian3.clone(currentPos, zMin);
            }

            if (z > zMax.z) {
                Cartesian3.clone(currentPos, zMax);
            }
        }

        // Compute x-, y-, and z-spans (Squared distances b/n each component's min. and max.).
        var xSpan = Cartesian3.magnitudeSquared(Cartesian3.subtract(xMax, xMin, fromPointsScratch));
        var ySpan = Cartesian3.magnitudeSquared(Cartesian3.subtract(yMax, yMin, fromPointsScratch));
        var zSpan = Cartesian3.magnitudeSquared(Cartesian3.subtract(zMax, zMin, fromPointsScratch));

        // Set the diameter endpoints to the largest span.
        var diameter1 = xMin;
        var diameter2 = xMax;
        var maxSpan = xSpan;
        if (ySpan > maxSpan) {
            maxSpan = ySpan;
            diameter1 = yMin;
            diameter2 = yMax;
        }
        if (zSpan > maxSpan) {
            maxSpan = zSpan;
            diameter1 = zMin;
            diameter2 = zMax;
        }

        // Calculate the center of the initial sphere found by Ritter's algorithm
        var ritterCenter = fromPointsRitterCenter;
        ritterCenter.x = (diameter1.x + diameter2.x) * 0.5;
        ritterCenter.y = (diameter1.y + diameter2.y) * 0.5;
        ritterCenter.z = (diameter1.z + diameter2.z) * 0.5;

        // Calculate the radius of the initial sphere found by Ritter's algorithm
        var radiusSquared = Cartesian3.magnitudeSquared(Cartesian3.subtract(diameter2, ritterCenter, fromPointsScratch));
        var ritterRadius = Math.sqrt(radiusSquared);

        // Find the center of the sphere found using the Naive method.
        var minBoxPt = fromPointsMinBoxPt;
        minBoxPt.x = xMin.x;
        minBoxPt.y = yMin.y;
        minBoxPt.z = zMin.z;

        var maxBoxPt = fromPointsMaxBoxPt;
        maxBoxPt.x = xMax.x;
        maxBoxPt.y = yMax.y;
        maxBoxPt.z = zMax.z;

        var naiveCenter = Cartesian3.midpoint(minBoxPt, maxBoxPt, fromPointsNaiveCenterScratch);

        // Begin 2nd pass to find naive radius and modify the ritter sphere.
        var naiveRadius = 0;
        for (i = 0; i < numElements; i += stride) {
            currentPos.x = positions[i] + center.x;
            currentPos.y = positions[i + 1] + center.y;
            currentPos.z = positions[i + 2] + center.z;

            // Find the furthest point from the naive center to calculate the naive radius.
            var r = Cartesian3.magnitude(Cartesian3.subtract(currentPos, naiveCenter, fromPointsScratch));
            if (r > naiveRadius) {
                naiveRadius = r;
            }

            // Make adjustments to the Ritter Sphere to include all points.
            var oldCenterToPointSquared = Cartesian3.magnitudeSquared(Cartesian3.subtract(currentPos, ritterCenter, fromPointsScratch));
            if (oldCenterToPointSquared > radiusSquared) {
                var oldCenterToPoint = Math.sqrt(oldCenterToPointSquared);
                // Calculate new radius to include the point that lies outside
                ritterRadius = (ritterRadius + oldCenterToPoint) * 0.5;
                radiusSquared = ritterRadius * ritterRadius;
                // Calculate center of new Ritter sphere
                var oldToNew = oldCenterToPoint - ritterRadius;
                ritterCenter.x = (ritterRadius * ritterCenter.x + oldToNew * currentPos.x) / oldCenterToPoint;
                ritterCenter.y = (ritterRadius * ritterCenter.y + oldToNew * currentPos.y) / oldCenterToPoint;
                ritterCenter.z = (ritterRadius * ritterCenter.z + oldToNew * currentPos.z) / oldCenterToPoint;
            }
        }

        if (ritterRadius < naiveRadius) {
            Cartesian3.clone(ritterCenter, result.center);
            result.radius = ritterRadius;
        } else {
            Cartesian3.clone(naiveCenter, result.center);
            result.radius = naiveRadius;
        }

        return result;
    };

    /**
     * Computes a tight-fitting bounding sphere enclosing a list of {@link EncodedCartesian3}s, where the points are
     * stored in parallel flat arrays in X, Y, Z, order.  The bounding sphere is computed by running two
     * algorithms, a naive algorithm and Ritter's algorithm. The smaller of the two spheres is used to
     * ensure a tight fit.
     *
     * @param {Number[]} [positionsHigh] An array of high bits of the encoded cartesians that the bounding sphere will enclose.  Each point
     *        is formed from three elements in the array in the order X, Y, Z.
     * @param {Number[]} [positionsLow] An array of low bits of the encoded cartesians that the bounding sphere will enclose.  Each point
     *        is formed from three elements in the array in the order X, Y, Z.
     * @param {BoundingSphere} [result] The object onto which to store the result.
     * @returns {BoundingSphere} The modified result parameter or a new BoundingSphere instance if one was not provided.
     *
     * @see {@link http://blogs.agi.com/insight3d/index.php/2008/02/04/a-bounding/|Bounding Sphere computation article}
     */
    BoundingSphere.fromEncodedCartesianVertices = function(positionsHigh, positionsLow, result) {
        if (!defined(result)) {
            result = new BoundingSphere();
        }

        if (!defined(positionsHigh) || !defined(positionsLow) || positionsHigh.length !== positionsLow.length || positionsHigh.length === 0) {
            result.center = Cartesian3.clone(Cartesian3.ZERO, result.center);
            result.radius = 0.0;
            return result;
        }

        var currentPos = fromPointsCurrentPos;
        currentPos.x = positionsHigh[0] + positionsLow[0];
        currentPos.y = positionsHigh[1] + positionsLow[1];
        currentPos.z = positionsHigh[2] + positionsLow[2];

        var xMin = Cartesian3.clone(currentPos, fromPointsXMin);
        var yMin = Cartesian3.clone(currentPos, fromPointsYMin);
        var zMin = Cartesian3.clone(currentPos, fromPointsZMin);

        var xMax = Cartesian3.clone(currentPos, fromPointsXMax);
        var yMax = Cartesian3.clone(currentPos, fromPointsYMax);
        var zMax = Cartesian3.clone(currentPos, fromPointsZMax);

        var numElements = positionsHigh.length;
        var i;
        for (i = 0; i < numElements; i += 3) {
            var x = positionsHigh[i] + positionsLow[i];
            var y = positionsHigh[i + 1] + positionsLow[i + 1];
            var z = positionsHigh[i + 2] + positionsLow[i + 2];

            currentPos.x = x;
            currentPos.y = y;
            currentPos.z = z;

            // Store points containing the the smallest and largest components
            if (x < xMin.x) {
                Cartesian3.clone(currentPos, xMin);
            }

            if (x > xMax.x) {
                Cartesian3.clone(currentPos, xMax);
            }

            if (y < yMin.y) {
                Cartesian3.clone(currentPos, yMin);
            }

            if (y > yMax.y) {
                Cartesian3.clone(currentPos, yMax);
            }

            if (z < zMin.z) {
                Cartesian3.clone(currentPos, zMin);
            }

            if (z > zMax.z) {
                Cartesian3.clone(currentPos, zMax);
            }
        }

        // Compute x-, y-, and z-spans (Squared distances b/n each component's min. and max.).
        var xSpan = Cartesian3.magnitudeSquared(Cartesian3.subtract(xMax, xMin, fromPointsScratch));
        var ySpan = Cartesian3.magnitudeSquared(Cartesian3.subtract(yMax, yMin, fromPointsScratch));
        var zSpan = Cartesian3.magnitudeSquared(Cartesian3.subtract(zMax, zMin, fromPointsScratch));

        // Set the diameter endpoints to the largest span.
        var diameter1 = xMin;
        var diameter2 = xMax;
        var maxSpan = xSpan;
        if (ySpan > maxSpan) {
            maxSpan = ySpan;
            diameter1 = yMin;
            diameter2 = yMax;
        }
        if (zSpan > maxSpan) {
            maxSpan = zSpan;
            diameter1 = zMin;
            diameter2 = zMax;
        }

        // Calculate the center of the initial sphere found by Ritter's algorithm
        var ritterCenter = fromPointsRitterCenter;
        ritterCenter.x = (diameter1.x + diameter2.x) * 0.5;
        ritterCenter.y = (diameter1.y + diameter2.y) * 0.5;
        ritterCenter.z = (diameter1.z + diameter2.z) * 0.5;

        // Calculate the radius of the initial sphere found by Ritter's algorithm
        var radiusSquared = Cartesian3.magnitudeSquared(Cartesian3.subtract(diameter2, ritterCenter, fromPointsScratch));
        var ritterRadius = Math.sqrt(radiusSquared);

        // Find the center of the sphere found using the Naive method.
        var minBoxPt = fromPointsMinBoxPt;
        minBoxPt.x = xMin.x;
        minBoxPt.y = yMin.y;
        minBoxPt.z = zMin.z;

        var maxBoxPt = fromPointsMaxBoxPt;
        maxBoxPt.x = xMax.x;
        maxBoxPt.y = yMax.y;
        maxBoxPt.z = zMax.z;

        var naiveCenter = Cartesian3.midpoint(minBoxPt, maxBoxPt, fromPointsNaiveCenterScratch);

        // Begin 2nd pass to find naive radius and modify the ritter sphere.
        var naiveRadius = 0;
        for (i = 0; i < numElements; i += 3) {
            currentPos.x = positionsHigh[i] + positionsLow[i];
            currentPos.y = positionsHigh[i + 1] + positionsLow[i + 1];
            currentPos.z = positionsHigh[i + 2] + positionsLow[i + 2];

            // Find the furthest point from the naive center to calculate the naive radius.
            var r = Cartesian3.magnitude(Cartesian3.subtract(currentPos, naiveCenter, fromPointsScratch));
            if (r > naiveRadius) {
                naiveRadius = r;
            }

            // Make adjustments to the Ritter Sphere to include all points.
            var oldCenterToPointSquared = Cartesian3.magnitudeSquared(Cartesian3.subtract(currentPos, ritterCenter, fromPointsScratch));
            if (oldCenterToPointSquared > radiusSquared) {
                var oldCenterToPoint = Math.sqrt(oldCenterToPointSquared);
                // Calculate new radius to include the point that lies outside
                ritterRadius = (ritterRadius + oldCenterToPoint) * 0.5;
                radiusSquared = ritterRadius * ritterRadius;
                // Calculate center of new Ritter sphere
                var oldToNew = oldCenterToPoint - ritterRadius;
                ritterCenter.x = (ritterRadius * ritterCenter.x + oldToNew * currentPos.x) / oldCenterToPoint;
                ritterCenter.y = (ritterRadius * ritterCenter.y + oldToNew * currentPos.y) / oldCenterToPoint;
                ritterCenter.z = (ritterRadius * ritterCenter.z + oldToNew * currentPos.z) / oldCenterToPoint;
            }
        }

        if (ritterRadius < naiveRadius) {
            Cartesian3.clone(ritterCenter, result.center);
            result.radius = ritterRadius;
        } else {
            Cartesian3.clone(naiveCenter, result.center);
            result.radius = naiveRadius;
        }

        return result;
    };

    /**
     * Computes a bounding sphere from the corner points of an axis-aligned bounding box.  The sphere
     * tighly and fully encompases the box.
     *
     * @param {Cartesian3} [corner] The minimum height over the rectangle.
     * @param {Cartesian3} [oppositeCorner] The maximum height over the rectangle.
     * @param {BoundingSphere} [result] The object onto which to store the result.
     * @returns {BoundingSphere} The modified result parameter or a new BoundingSphere instance if none was provided.
     *
     * @example
     * // Create a bounding sphere around the unit cube
     * var sphere = Cesium.BoundingSphere.fromCornerPoints(new Cesium.Cartesian3(-0.5, -0.5, -0.5), new Cesium.Cartesian3(0.5, 0.5, 0.5));
     */
    BoundingSphere.fromCornerPoints = function(corner, oppositeCorner, result) {
                Check.typeOf.object('corner', corner);
        Check.typeOf.object('oppositeCorner', oppositeCorner);
        
        if (!defined(result)) {
            result = new BoundingSphere();
        }

        var center = Cartesian3.midpoint(corner, oppositeCorner, result.center);
        result.radius = Cartesian3.distance(center, oppositeCorner);
        return result;
    };

    /**
     * Creates a bounding sphere encompassing an ellipsoid.
     *
     * @param {Ellipsoid} ellipsoid The ellipsoid around which to create a bounding sphere.
     * @param {BoundingSphere} [result] The object onto which to store the result.
     * @returns {BoundingSphere} The modified result parameter or a new BoundingSphere instance if none was provided.
     *
     * @example
     * var boundingSphere = Cesium.BoundingSphere.fromEllipsoid(ellipsoid);
     */
    BoundingSphere.fromEllipsoid = function(ellipsoid, result) {
                Check.typeOf.object('ellipsoid', ellipsoid);
        
        if (!defined(result)) {
            result = new BoundingSphere();
        }

        Cartesian3.clone(Cartesian3.ZERO, result.center);
        result.radius = ellipsoid.maximumRadius;
        return result;
    };

    var fromBoundingSpheresScratch = new Cartesian3();

    /**
     * Computes a tight-fitting bounding sphere enclosing the provided array of bounding spheres.
     *
     * @param {BoundingSphere[]} [boundingSpheres] The array of bounding spheres.
     * @param {BoundingSphere} [result] The object onto which to store the result.
     * @returns {BoundingSphere} The modified result parameter or a new BoundingSphere instance if none was provided.
     */
    BoundingSphere.fromBoundingSpheres = function(boundingSpheres, result) {
        if (!defined(result)) {
            result = new BoundingSphere();
        }

        if (!defined(boundingSpheres) || boundingSpheres.length === 0) {
            result.center = Cartesian3.clone(Cartesian3.ZERO, result.center);
            result.radius = 0.0;
            return result;
        }

        var length = boundingSpheres.length;
        if (length === 1) {
            return BoundingSphere.clone(boundingSpheres[0], result);
        }

        if (length === 2) {
            return BoundingSphere.union(boundingSpheres[0], boundingSpheres[1], result);
        }

        var positions = [];
        var i;
        for (i = 0; i < length; i++) {
            positions.push(boundingSpheres[i].center);
        }

        result = BoundingSphere.fromPoints(positions, result);

        var center = result.center;
        var radius = result.radius;
        for (i = 0; i < length; i++) {
            var tmp = boundingSpheres[i];
            radius = Math.max(radius, Cartesian3.distance(center, tmp.center, fromBoundingSpheresScratch) + tmp.radius);
        }
        result.radius = radius;

        return result;
    };

    var fromOrientedBoundingBoxScratchU = new Cartesian3();
    var fromOrientedBoundingBoxScratchV = new Cartesian3();
    var fromOrientedBoundingBoxScratchW = new Cartesian3();

    /**
     * Computes a tight-fitting bounding sphere enclosing the provided oriented bounding box.
     *
     * @param {OrientedBoundingBox} orientedBoundingBox The oriented bounding box.
     * @param {BoundingSphere} [result] The object onto which to store the result.
     * @returns {BoundingSphere} The modified result parameter or a new BoundingSphere instance if none was provided.
     */
    BoundingSphere.fromOrientedBoundingBox = function(orientedBoundingBox, result) {
                Check.defined('orientedBoundingBox', orientedBoundingBox);
        
        if (!defined(result)) {
            result = new BoundingSphere();
        }

        var halfAxes = orientedBoundingBox.halfAxes;
        var u = Matrix3.getColumn(halfAxes, 0, fromOrientedBoundingBoxScratchU);
        var v = Matrix3.getColumn(halfAxes, 1, fromOrientedBoundingBoxScratchV);
        var w = Matrix3.getColumn(halfAxes, 2, fromOrientedBoundingBoxScratchW);

        Cartesian3.add(u, v, u);
        Cartesian3.add(u, w, u);

        result.center = Cartesian3.clone(orientedBoundingBox.center, result.center);
        result.radius = Cartesian3.magnitude(u);

        return result;
    };

    /**
     * Duplicates a BoundingSphere instance.
     *
     * @param {BoundingSphere} sphere The bounding sphere to duplicate.
     * @param {BoundingSphere} [result] The object onto which to store the result.
     * @returns {BoundingSphere} The modified result parameter or a new BoundingSphere instance if none was provided. (Returns undefined if sphere is undefined)
     */
    BoundingSphere.clone = function(sphere, result) {
        if (!defined(sphere)) {
            return undefined;
        }

        if (!defined(result)) {
            return new BoundingSphere(sphere.center, sphere.radius);
        }

        result.center = Cartesian3.clone(sphere.center, result.center);
        result.radius = sphere.radius;
        return result;
    };

    /**
     * The number of elements used to pack the object into an array.
     * @type {Number}
     */
    BoundingSphere.packedLength = 4;

    /**
     * Stores the provided instance into the provided array.
     *
     * @param {BoundingSphere} value The value to pack.
     * @param {Number[]} array The array to pack into.
     * @param {Number} [startingIndex=0] The index into the array at which to start packing the elements.
     *
     * @returns {Number[]} The array that was packed into
     */
    BoundingSphere.pack = function(value, array, startingIndex) {
                Check.typeOf.object('value', value);
        Check.defined('array', array);
        
        startingIndex = defaultValue(startingIndex, 0);

        var center = value.center;
        array[startingIndex++] = center.x;
        array[startingIndex++] = center.y;
        array[startingIndex++] = center.z;
        array[startingIndex] = value.radius;

        return array;
    };

    /**
     * Retrieves an instance from a packed array.
     *
     * @param {Number[]} array The packed array.
     * @param {Number} [startingIndex=0] The starting index of the element to be unpacked.
     * @param {BoundingSphere} [result] The object into which to store the result.
     * @returns {BoundingSphere} The modified result parameter or a new BoundingSphere instance if one was not provided.
     */
    BoundingSphere.unpack = function(array, startingIndex, result) {
                Check.defined('array', array);
        
        startingIndex = defaultValue(startingIndex, 0);

        if (!defined(result)) {
            result = new BoundingSphere();
        }

        var center = result.center;
        center.x = array[startingIndex++];
        center.y = array[startingIndex++];
        center.z = array[startingIndex++];
        result.radius = array[startingIndex];
        return result;
    };

    var unionScratch = new Cartesian3();
    var unionScratchCenter = new Cartesian3();
    /**
     * Computes a bounding sphere that contains both the left and right bounding spheres.
     *
     * @param {BoundingSphere} left A sphere to enclose in a bounding sphere.
     * @param {BoundingSphere} right A sphere to enclose in a bounding sphere.
     * @param {BoundingSphere} [result] The object onto which to store the result.
     * @returns {BoundingSphere} The modified result parameter or a new BoundingSphere instance if none was provided.
     */
    BoundingSphere.union = function(left, right, result) {
                Check.typeOf.object('left', left);
        Check.typeOf.object('right', right);
        
        if (!defined(result)) {
            result = new BoundingSphere();
        }

        var leftCenter = left.center;
        var leftRadius = left.radius;
        var rightCenter = right.center;
        var rightRadius = right.radius;

        var toRightCenter = Cartesian3.subtract(rightCenter, leftCenter, unionScratch);
        var centerSeparation = Cartesian3.magnitude(toRightCenter);

        if (leftRadius >= (centerSeparation + rightRadius)) {
            // Left sphere wins.
            left.clone(result);
            return result;
        }

        if (rightRadius >= (centerSeparation + leftRadius)) {
            // Right sphere wins.
            right.clone(result);
            return result;
        }

        // There are two tangent points, one on far side of each sphere.
        var halfDistanceBetweenTangentPoints = (leftRadius + centerSeparation + rightRadius) * 0.5;

        // Compute the center point halfway between the two tangent points.
        var center = Cartesian3.multiplyByScalar(toRightCenter,
                (-leftRadius + halfDistanceBetweenTangentPoints) / centerSeparation, unionScratchCenter);
        Cartesian3.add(center, leftCenter, center);
        Cartesian3.clone(center, result.center);
        result.radius = halfDistanceBetweenTangentPoints;

        return result;
    };

    var expandScratch = new Cartesian3();
    /**
     * Computes a bounding sphere by enlarging the provided sphere to contain the provided point.
     *
     * @param {BoundingSphere} sphere A sphere to expand.
     * @param {Cartesian3} point A point to enclose in a bounding sphere.
     * @param {BoundingSphere} [result] The object onto which to store the result.
     * @returns {BoundingSphere} The modified result parameter or a new BoundingSphere instance if none was provided.
     */
    BoundingSphere.expand = function(sphere, point, result) {
                Check.typeOf.object('sphere', sphere);
        Check.typeOf.object('point', point);
        
        result = BoundingSphere.clone(sphere, result);

        var radius = Cartesian3.magnitude(Cartesian3.subtract(point, result.center, expandScratch));
        if (radius > result.radius) {
            result.radius = radius;
        }

        return result;
    };

    /**
     * Determines which side of a plane a sphere is located.
     *
     * @param {BoundingSphere} sphere The bounding sphere to test.
     * @param {Plane} plane The plane to test against.
     * @returns {Intersect} {@link Intersect.INSIDE} if the entire sphere is on the side of the plane
     *                      the normal is pointing, {@link Intersect.OUTSIDE} if the entire sphere is
     *                      on the opposite side, and {@link Intersect.INTERSECTING} if the sphere
     *                      intersects the plane.
     */
    BoundingSphere.intersectPlane = function(sphere, plane) {
                Check.typeOf.object('sphere', sphere);
        Check.typeOf.object('plane', plane);
        
        var center = sphere.center;
        var radius = sphere.radius;
        var normal = plane.normal;
        var distanceToPlane = Cartesian3.dot(normal, center) + plane.distance;

        if (distanceToPlane < -radius) {
            // The center point is negative side of the plane normal
            return Intersect.OUTSIDE;
        } else if (distanceToPlane < radius) {
            // The center point is positive side of the plane, but radius extends beyond it; partial overlap
            return Intersect.INTERSECTING;
        }
        return Intersect.INSIDE;
    };

    /**
     * Applies a 4x4 affine transformation matrix to a bounding sphere.
     *
     * @param {BoundingSphere} sphere The bounding sphere to apply the transformation to.
     * @param {Matrix4} transform The transformation matrix to apply to the bounding sphere.
     * @param {BoundingSphere} [result] The object onto which to store the result.
     * @returns {BoundingSphere} The modified result parameter or a new BoundingSphere instance if none was provided.
     */
    BoundingSphere.transform = function(sphere, transform, result) {
                Check.typeOf.object('sphere', sphere);
        Check.typeOf.object('transform', transform);
        
        if (!defined(result)) {
            result = new BoundingSphere();
        }

        result.center = Matrix4.multiplyByPoint(transform, sphere.center, result.center);
        result.radius = Matrix4.getMaximumScale(transform) * sphere.radius;

        return result;
    };

    var distanceSquaredToScratch = new Cartesian3();

    /**
     * Computes the estimated distance squared from the closest point on a bounding sphere to a point.
     *
     * @param {BoundingSphere} sphere The sphere.
     * @param {Cartesian3} cartesian The point
     * @returns {Number} The estimated distance squared from the bounding sphere to the point.
     *
     * @example
     * // Sort bounding spheres from back to front
     * spheres.sort(function(a, b) {
     *     return Cesium.BoundingSphere.distanceSquaredTo(b, camera.positionWC) - Cesium.BoundingSphere.distanceSquaredTo(a, camera.positionWC);
     * });
     */
    BoundingSphere.distanceSquaredTo = function(sphere, cartesian) {
                Check.typeOf.object('sphere', sphere);
        Check.typeOf.object('cartesian', cartesian);
        
        var diff = Cartesian3.subtract(sphere.center, cartesian, distanceSquaredToScratch);
        return Cartesian3.magnitudeSquared(diff) - sphere.radius * sphere.radius;
    };

    /**
     * Applies a 4x4 affine transformation matrix to a bounding sphere where there is no scale
     * The transformation matrix is not verified to have a uniform scale of 1.
     * This method is faster than computing the general bounding sphere transform using {@link BoundingSphere.transform}.
     *
     * @param {BoundingSphere} sphere The bounding sphere to apply the transformation to.
     * @param {Matrix4} transform The transformation matrix to apply to the bounding sphere.
     * @param {BoundingSphere} [result] The object onto which to store the result.
     * @returns {BoundingSphere} The modified result parameter or a new BoundingSphere instance if none was provided.
     *
     * @example
     * var modelMatrix = Cesium.Transforms.eastNorthUpToFixedFrame(positionOnEllipsoid);
     * var boundingSphere = new Cesium.BoundingSphere();
     * var newBoundingSphere = Cesium.BoundingSphere.transformWithoutScale(boundingSphere, modelMatrix);
     */
    BoundingSphere.transformWithoutScale = function(sphere, transform, result) {
                Check.typeOf.object('sphere', sphere);
        Check.typeOf.object('transform', transform);
        
        if (!defined(result)) {
            result = new BoundingSphere();
        }

        result.center = Matrix4.multiplyByPoint(transform, sphere.center, result.center);
        result.radius = sphere.radius;

        return result;
    };

    var scratchCartesian3 = new Cartesian3();
    /**
     * The distances calculated by the vector from the center of the bounding sphere to position projected onto direction
     * plus/minus the radius of the bounding sphere.
     * <br>
     * If you imagine the infinite number of planes with normal direction, this computes the smallest distance to the
     * closest and farthest planes from position that intersect the bounding sphere.
     *
     * @param {BoundingSphere} sphere The bounding sphere to calculate the distance to.
     * @param {Cartesian3} position The position to calculate the distance from.
     * @param {Cartesian3} direction The direction from position.
     * @param {Interval} [result] A Interval to store the nearest and farthest distances.
     * @returns {Interval} The nearest and farthest distances on the bounding sphere from position in direction.
     */
    BoundingSphere.computePlaneDistances = function(sphere, position, direction, result) {
                Check.typeOf.object('sphere', sphere);
        Check.typeOf.object('position', position);
        Check.typeOf.object('direction', direction);
        
        if (!defined(result)) {
            result = new Interval();
        }

        var toCenter = Cartesian3.subtract(sphere.center, position, scratchCartesian3);
        var mag = Cartesian3.dot(direction, toCenter);

        result.start = mag - sphere.radius;
        result.stop = mag + sphere.radius;
        return result;
    };

    var projectTo2DNormalScratch = new Cartesian3();
    var projectTo2DEastScratch = new Cartesian3();
    var projectTo2DNorthScratch = new Cartesian3();
    var projectTo2DWestScratch = new Cartesian3();
    var projectTo2DSouthScratch = new Cartesian3();
    var projectTo2DCartographicScratch = new Cartographic();
    var projectTo2DPositionsScratch = new Array(8);
    for (var n = 0; n < 8; ++n) {
        projectTo2DPositionsScratch[n] = new Cartesian3();
    }

    var projectTo2DProjection = new GeographicProjection();
    /**
     * Creates a bounding sphere in 2D from a bounding sphere in 3D world coordinates.
     *
     * @param {BoundingSphere} sphere The bounding sphere to transform to 2D.
     * @param {Object} [projection=GeographicProjection] The projection to 2D.
     * @param {BoundingSphere} [result] The object onto which to store the result.
     * @returns {BoundingSphere} The modified result parameter or a new BoundingSphere instance if none was provided.
     */
    BoundingSphere.projectTo2D = function(sphere, projection, result) {
                Check.typeOf.object('sphere', sphere);
        
        projection = defaultValue(projection, projectTo2DProjection);

        var ellipsoid = projection.ellipsoid;
        var center = sphere.center;
        var radius = sphere.radius;

        var normal = ellipsoid.geodeticSurfaceNormal(center, projectTo2DNormalScratch);
        var east = Cartesian3.cross(Cartesian3.UNIT_Z, normal, projectTo2DEastScratch);
        Cartesian3.normalize(east, east);
        var north = Cartesian3.cross(normal, east, projectTo2DNorthScratch);
        Cartesian3.normalize(north, north);

        Cartesian3.multiplyByScalar(normal, radius, normal);
        Cartesian3.multiplyByScalar(north, radius, north);
        Cartesian3.multiplyByScalar(east, radius, east);

        var south = Cartesian3.negate(north, projectTo2DSouthScratch);
        var west = Cartesian3.negate(east, projectTo2DWestScratch);

        var positions = projectTo2DPositionsScratch;

        // top NE corner
        var corner = positions[0];
        Cartesian3.add(normal, north, corner);
        Cartesian3.add(corner, east, corner);

        // top NW corner
        corner = positions[1];
        Cartesian3.add(normal, north, corner);
        Cartesian3.add(corner, west, corner);

        // top SW corner
        corner = positions[2];
        Cartesian3.add(normal, south, corner);
        Cartesian3.add(corner, west, corner);

        // top SE corner
        corner = positions[3];
        Cartesian3.add(normal, south, corner);
        Cartesian3.add(corner, east, corner);

        Cartesian3.negate(normal, normal);

        // bottom NE corner
        corner = positions[4];
        Cartesian3.add(normal, north, corner);
        Cartesian3.add(corner, east, corner);

        // bottom NW corner
        corner = positions[5];
        Cartesian3.add(normal, north, corner);
        Cartesian3.add(corner, west, corner);

        // bottom SW corner
        corner = positions[6];
        Cartesian3.add(normal, south, corner);
        Cartesian3.add(corner, west, corner);

        // bottom SE corner
        corner = positions[7];
        Cartesian3.add(normal, south, corner);
        Cartesian3.add(corner, east, corner);

        var length = positions.length;
        for (var i = 0; i < length; ++i) {
            var position = positions[i];
            Cartesian3.add(center, position, position);
            var cartographic = ellipsoid.cartesianToCartographic(position, projectTo2DCartographicScratch);
            projection.project(cartographic, position);
        }

        result = BoundingSphere.fromPoints(positions, result);

        // swizzle center components
        center = result.center;
        var x = center.x;
        var y = center.y;
        var z = center.z;
        center.x = z;
        center.y = x;
        center.z = y;

        return result;
    };

    /**
     * Determines whether or not a sphere is hidden from view by the occluder.
     *
     * @param {BoundingSphere} sphere The bounding sphere surrounding the occludee object.
     * @param {Occluder} occluder The occluder.
     * @returns {Boolean} <code>true</code> if the sphere is not visible; otherwise <code>false</code>.
     */
    BoundingSphere.isOccluded = function(sphere, occluder) {
                Check.typeOf.object('sphere', sphere);
        Check.typeOf.object('occluder', occluder);
                return !occluder.isBoundingSphereVisible(sphere);
    };

    /**
     * Compares the provided BoundingSphere componentwise and returns
     * <code>true</code> if they are equal, <code>false</code> otherwise.
     *
     * @param {BoundingSphere} [left] The first BoundingSphere.
     * @param {BoundingSphere} [right] The second BoundingSphere.
     * @returns {Boolean} <code>true</code> if left and right are equal, <code>false</code> otherwise.
     */
    BoundingSphere.equals = function(left, right) {
        return (left === right) ||
               ((defined(left)) &&
                (defined(right)) &&
                Cartesian3.equals(left.center, right.center) &&
                left.radius === right.radius);
    };

    /**
     * Determines which side of a plane the sphere is located.
     *
     * @param {Plane} plane The plane to test against.
     * @returns {Intersect} {@link Intersect.INSIDE} if the entire sphere is on the side of the plane
     *                      the normal is pointing, {@link Intersect.OUTSIDE} if the entire sphere is
     *                      on the opposite side, and {@link Intersect.INTERSECTING} if the sphere
     *                      intersects the plane.
     */
    BoundingSphere.prototype.intersectPlane = function(plane) {
        return BoundingSphere.intersectPlane(this, plane);
    };

    /**
     * Computes the estimated distance squared from the closest point on a bounding sphere to a point.
     *
     * @param {Cartesian3} cartesian The point
     * @returns {Number} The estimated distance squared from the bounding sphere to the point.
     *
     * @example
     * // Sort bounding spheres from back to front
     * spheres.sort(function(a, b) {
     *     return b.distanceSquaredTo(camera.positionWC) - a.distanceSquaredTo(camera.positionWC);
     * });
     */
    BoundingSphere.prototype.distanceSquaredTo = function(cartesian) {
        return BoundingSphere.distanceSquaredTo(this, cartesian);
    };

    /**
     * The distances calculated by the vector from the center of the bounding sphere to position projected onto direction
     * plus/minus the radius of the bounding sphere.
     * <br>
     * If you imagine the infinite number of planes with normal direction, this computes the smallest distance to the
     * closest and farthest planes from position that intersect the bounding sphere.
     *
     * @param {Cartesian3} position The position to calculate the distance from.
     * @param {Cartesian3} direction The direction from position.
     * @param {Interval} [result] A Interval to store the nearest and farthest distances.
     * @returns {Interval} The nearest and farthest distances on the bounding sphere from position in direction.
     */
    BoundingSphere.prototype.computePlaneDistances = function(position, direction, result) {
        return BoundingSphere.computePlaneDistances(this, position, direction, result);
    };

    /**
     * Determines whether or not a sphere is hidden from view by the occluder.
     *
     * @param {Occluder} occluder The occluder.
     * @returns {Boolean} <code>true</code> if the sphere is not visible; otherwise <code>false</code>.
     */
    BoundingSphere.prototype.isOccluded = function(occluder) {
        return BoundingSphere.isOccluded(this, occluder);
    };

    /**
     * Compares this BoundingSphere against the provided BoundingSphere componentwise and returns
     * <code>true</code> if they are equal, <code>false</code> otherwise.
     *
     * @param {BoundingSphere} [right] The right hand side BoundingSphere.
     * @returns {Boolean} <code>true</code> if they are equal, <code>false</code> otherwise.
     */
    BoundingSphere.prototype.equals = function(right) {
        return BoundingSphere.equals(this, right);
    };

    /**
     * Duplicates this BoundingSphere instance.
     *
     * @param {BoundingSphere} [result] The object onto which to store the result.
     * @returns {BoundingSphere} The modified result parameter or a new BoundingSphere instance if none was provided.
     */
    BoundingSphere.prototype.clone = function(result) {
        return BoundingSphere.clone(this, result);
    };

    /**
     * Computes the radius of the BoundingSphere.
     * @returns {Number} The radius of the BoundingSphere.
     */
    BoundingSphere.prototype.volume = function() {
        var radius = this.radius;
        return volumeConstant * radius * radius * radius;
    };

    return BoundingSphere;
});

define('Core/Fullscreen',[
        './defined',
        './defineProperties'
    ], function(
        defined,
        defineProperties) {
    'use strict';

    var _supportsFullscreen;
    var _names = {
        requestFullscreen : undefined,
        exitFullscreen : undefined,
        fullscreenEnabled : undefined,
        fullscreenElement : undefined,
        fullscreenchange : undefined,
        fullscreenerror : undefined
    };

    /**
     * Browser-independent functions for working with the standard fullscreen API.
     *
     * @exports Fullscreen
     * @namespace
     *
     * @see {@link http://dvcs.w3.org/hg/fullscreen/raw-file/tip/Overview.html|W3C Fullscreen Living Specification}
     */
    var Fullscreen = {};

    defineProperties(Fullscreen, {
        /**
         * The element that is currently fullscreen, if any.  To simply check if the
         * browser is in fullscreen mode or not, use {@link Fullscreen#fullscreen}.
         * @memberof Fullscreen
         * @type {Object}
         * @readonly
         */
        element : {
            get : function() {
                if (!Fullscreen.supportsFullscreen()) {
                    return undefined;
                }

                return document[_names.fullscreenElement];
            }
        },

        /**
         * The name of the event on the document that is fired when fullscreen is
         * entered or exited.  This event name is intended for use with addEventListener.
         * In your event handler, to determine if the browser is in fullscreen mode or not,
         * use {@link Fullscreen#fullscreen}.
         * @memberof Fullscreen
         * @type {String}
         * @readonly
         */
        changeEventName : {
            get : function() {
                if (!Fullscreen.supportsFullscreen()) {
                    return undefined;
                }

                return _names.fullscreenchange;
            }
        },

        /**
         * The name of the event that is fired when a fullscreen error
         * occurs.  This event name is intended for use with addEventListener.
         * @memberof Fullscreen
         * @type {String}
         * @readonly
         */
        errorEventName : {
            get : function() {
                if (!Fullscreen.supportsFullscreen()) {
                    return undefined;
                }

                return _names.fullscreenerror;
            }
        },

        /**
         * Determine whether the browser will allow an element to be made fullscreen, or not.
         * For example, by default, iframes cannot go fullscreen unless the containing page
         * adds an "allowfullscreen" attribute (or prefixed equivalent).
         * @memberof Fullscreen
         * @type {Boolean}
         * @readonly
         */
        enabled : {
            get : function() {
                if (!Fullscreen.supportsFullscreen()) {
                    return undefined;
                }

                return document[_names.fullscreenEnabled];
            }
        },

        /**
         * Determines if the browser is currently in fullscreen mode.
         * @memberof Fullscreen
         * @type {Boolean}
         * @readonly
         */
        fullscreen : {
            get : function() {
                if (!Fullscreen.supportsFullscreen()) {
                    return undefined;
                }

                return Fullscreen.element !== null;
            }
        }
    });

    /**
     * Detects whether the browser supports the standard fullscreen API.
     *
     * @returns {Boolean} <code>true</code> if the browser supports the standard fullscreen API,
     * <code>false</code> otherwise.
     */
    Fullscreen.supportsFullscreen = function() {
        if (defined(_supportsFullscreen)) {
            return _supportsFullscreen;
        }

        _supportsFullscreen = false;

        var body = document.body;
        if (typeof body.requestFullscreen === 'function') {
            // go with the unprefixed, standard set of names
            _names.requestFullscreen = 'requestFullscreen';
            _names.exitFullscreen = 'exitFullscreen';
            _names.fullscreenEnabled = 'fullscreenEnabled';
            _names.fullscreenElement = 'fullscreenElement';
            _names.fullscreenchange = 'fullscreenchange';
            _names.fullscreenerror = 'fullscreenerror';
            _supportsFullscreen = true;
            return _supportsFullscreen;
        }

        //check for the correct combination of prefix plus the various names that browsers use
        var prefixes = ['webkit', 'moz', 'o', 'ms', 'khtml'];
        var name;
        for (var i = 0, len = prefixes.length; i < len; ++i) {
            var prefix = prefixes[i];

            // casing of Fullscreen differs across browsers
            name = prefix + 'RequestFullscreen';
            if (typeof body[name] === 'function') {
                _names.requestFullscreen = name;
                _supportsFullscreen = true;
            } else {
                name = prefix + 'RequestFullScreen';
                if (typeof body[name] === 'function') {
                    _names.requestFullscreen = name;
                    _supportsFullscreen = true;
                }
            }

            // disagreement about whether it's "exit" as per spec, or "cancel"
            name = prefix + 'ExitFullscreen';
            if (typeof document[name] === 'function') {
                _names.exitFullscreen = name;
            } else {
                name = prefix + 'CancelFullScreen';
                if (typeof document[name] === 'function') {
                    _names.exitFullscreen = name;
                }
            }

            // casing of Fullscreen differs across browsers
            name = prefix + 'FullscreenEnabled';
            if (document[name] !== undefined) {
                _names.fullscreenEnabled = name;
            } else {
                name = prefix + 'FullScreenEnabled';
                if (document[name] !== undefined) {
                    _names.fullscreenEnabled = name;
                }
            }

            // casing of Fullscreen differs across browsers
            name = prefix + 'FullscreenElement';
            if (document[name] !== undefined) {
                _names.fullscreenElement = name;
            } else {
                name = prefix + 'FullScreenElement';
                if (document[name] !== undefined) {
                    _names.fullscreenElement = name;
                }
            }

            // thankfully, event names are all lowercase per spec
            name = prefix + 'fullscreenchange';
            // event names do not have 'on' in the front, but the property on the document does
            if (document['on' + name] !== undefined) {
                //except on IE
                if (prefix === 'ms') {
                    name = 'MSFullscreenChange';
                }
                _names.fullscreenchange = name;
            }

            name = prefix + 'fullscreenerror';
            if (document['on' + name] !== undefined) {
                //except on IE
                if (prefix === 'ms') {
                    name = 'MSFullscreenError';
                }
                _names.fullscreenerror = name;
            }
        }

        return _supportsFullscreen;
    };

    /**
     * Asynchronously requests the browser to enter fullscreen mode on the given element.
     * If fullscreen mode is not supported by the browser, does nothing.
     *
     * @param {Object} element The HTML element which will be placed into fullscreen mode.
     * @param {HMDVRDevice} [vrDevice] The VR device.
     *
     * @example
     * // Put the entire page into fullscreen.
     * Cesium.Fullscreen.requestFullscreen(document.body)
     *
     * // Place only the Cesium canvas into fullscreen.
     * Cesium.Fullscreen.requestFullscreen(scene.canvas)
     */
    Fullscreen.requestFullscreen = function(element, vrDevice) {
        if (!Fullscreen.supportsFullscreen()) {
            return;
        }

        element[_names.requestFullscreen]({ vrDisplay: vrDevice });
    };

    /**
     * Asynchronously exits fullscreen mode.  If the browser is not currently
     * in fullscreen, or if fullscreen mode is not supported by the browser, does nothing.
     */
    Fullscreen.exitFullscreen = function() {
        if (!Fullscreen.supportsFullscreen()) {
            return;
        }

        document[_names.exitFullscreen]();
    };

    //For unit tests
    Fullscreen._names = _names;

    return Fullscreen;
});

/**
  @license
  when.js - https://github.com/cujojs/when

  MIT License (c) copyright B Cavalier & J Hann

 * A lightweight CommonJS Promises/A and when() implementation
 * when is part of the cujo.js family of libraries (http://cujojs.com/)
 *
 * Licensed under the MIT License at:
 * http://www.opensource.org/licenses/mit-license.php
 *
 * @version 1.7.1
 */

(function(define) { 'use strict';
define('ThirdParty/when',[],function () {
	var reduceArray, slice, undef;

	//
	// Public API
	//

	when.defer     = defer;     // Create a deferred
	when.resolve   = resolve;   // Create a resolved promise
	when.reject    = reject;    // Create a rejected promise

	when.join      = join;      // Join 2 or more promises

	when.all       = all;       // Resolve a list of promises
	when.map       = map;       // Array.map() for promises
	when.reduce    = reduce;    // Array.reduce() for promises

	when.any       = any;       // One-winner race
	when.some      = some;      // Multi-winner race

	when.chain     = chain;     // Make a promise trigger another resolver

	when.isPromise = isPromise; // Determine if a thing is a promise

	/**
	 * Register an observer for a promise or immediate value.
	 *
	 * @param {*} promiseOrValue
	 * @param {function?} [onFulfilled] callback to be called when promiseOrValue is
	 *   successfully fulfilled.  If promiseOrValue is an immediate value, callback
	 *   will be invoked immediately.
	 * @param {function?} [onRejected] callback to be called when promiseOrValue is
	 *   rejected.
	 * @param {function?} [onProgress] callback to be called when progress updates
	 *   are issued for promiseOrValue.
	 * @returns {Promise} a new {@link Promise} that will complete with the return
	 *   value of callback or errback or the completion value of promiseOrValue if
	 *   callback and/or errback is not supplied.
	 */
	function when(promiseOrValue, onFulfilled, onRejected, onProgress) {
		// Get a trusted promise for the input promiseOrValue, and then
		// register promise handlers
		return resolve(promiseOrValue).then(onFulfilled, onRejected, onProgress);
	}

	/**
	 * Returns promiseOrValue if promiseOrValue is a {@link Promise}, a new Promise if
	 * promiseOrValue is a foreign promise, or a new, already-fulfilled {@link Promise}
	 * whose value is promiseOrValue if promiseOrValue is an immediate value.
	 *
	 * @param {*} promiseOrValue
	 * @returns Guaranteed to return a trusted Promise.  If promiseOrValue is a when.js {@link Promise}
	 *   returns promiseOrValue, otherwise, returns a new, already-resolved, when.js {@link Promise}
	 *   whose resolution value is:
	 *   * the resolution value of promiseOrValue if it's a foreign promise, or
	 *   * promiseOrValue if it's a value
	 */
	function resolve(promiseOrValue) {
		var promise, deferred;

		if(promiseOrValue instanceof Promise) {
			// It's a when.js promise, so we trust it
			promise = promiseOrValue;

		} else {
			// It's not a when.js promise. See if it's a foreign promise or a value.
			if(isPromise(promiseOrValue)) {
				// It's a thenable, but we don't know where it came from, so don't trust
				// its implementation entirely.  Introduce a trusted middleman when.js promise
				deferred = defer();

				// IMPORTANT: This is the only place when.js should ever call .then() on an
				// untrusted promise. Don't expose the return value to the untrusted promise
				promiseOrValue.then(
					function(value)  { deferred.resolve(value); },
					function(reason) { deferred.reject(reason); },
					function(update) { deferred.progress(update); }
				);

				promise = deferred.promise;

			} else {
				// It's a value, not a promise.  Create a resolved promise for it.
				promise = fulfilled(promiseOrValue);
			}
		}

		return promise;
	}

	/**
	 * Returns a rejected promise for the supplied promiseOrValue.  The returned
	 * promise will be rejected with:
	 * - promiseOrValue, if it is a value, or
	 * - if promiseOrValue is a promise
	 *   - promiseOrValue's value after it is fulfilled
	 *   - promiseOrValue's reason after it is rejected
	 * @param {*} promiseOrValue the rejected value of the returned {@link Promise}
	 * @returns {Promise} rejected {@link Promise}
	 */
	function reject(promiseOrValue) {
		return when(promiseOrValue, rejected);
	}

	/**
	 * Trusted Promise constructor.  A Promise created from this constructor is
	 * a trusted when.js promise.  Any other duck-typed promise is considered
	 * untrusted.
	 * @constructor
	 * @name Promise
	 */
	function Promise(then) {
		this.then = then;
	}

	Promise.prototype = {
		/**
		 * Register a callback that will be called when a promise is
		 * fulfilled or rejected.  Optionally also register a progress handler.
		 * Shortcut for .then(onFulfilledOrRejected, onFulfilledOrRejected, onProgress)
		 * @param {function?} [onFulfilledOrRejected]
		 * @param {function?} [onProgress]
		 * @returns {Promise}
		 */
		always: function(onFulfilledOrRejected, onProgress) {
			return this.then(onFulfilledOrRejected, onFulfilledOrRejected, onProgress);
		},

		/**
		 * Register a rejection handler.  Shortcut for .then(undefined, onRejected)
		 * @param {function?} onRejected
		 * @returns {Promise}
		 */
		otherwise: function(onRejected) {
			return this.then(undef, onRejected);
		},

		/**
		 * Shortcut for .then(function() { return value; })
		 * @param  {*} value
		 * @returns {Promise} a promise that:
		 *  - is fulfilled if value is not a promise, or
		 *  - if value is a promise, will fulfill with its value, or reject
		 *    with its reason.
		 */
		yield: function(value) {
			return this.then(function() {
				return value;
			});
		},

		/**
		 * Assumes that this promise will fulfill with an array, and arranges
		 * for the onFulfilled to be called with the array as its argument list
		 * i.e. onFulfilled.spread(undefined, array).
		 * @param {function} onFulfilled function to receive spread arguments
		 * @returns {Promise}
		 */
		spread: function(onFulfilled) {
			return this.then(function(array) {
				// array may contain promises, so resolve its contents.
				return all(array, function(array) {
					return onFulfilled.apply(undef, array);
				});
			});
		}
	};

	/**
	 * Create an already-resolved promise for the supplied value
	 * @private
	 *
	 * @param {*} value
	 * @returns {Promise} fulfilled promise
	 */
	function fulfilled(value) {
		var p = new Promise(function(onFulfilled) {
			// TODO: Promises/A+ check typeof onFulfilled
			try {
				return resolve(onFulfilled ? onFulfilled(value) : value);
			} catch(e) {
				return rejected(e);
			}
		});

		return p;
	}

	/**
	 * Create an already-rejected {@link Promise} with the supplied
	 * rejection reason.
	 * @private
	 *
	 * @param {*} reason
	 * @returns {Promise} rejected promise
	 */
	function rejected(reason) {
		var p = new Promise(function(_, onRejected) {
			// TODO: Promises/A+ check typeof onRejected
			try {
				return onRejected ? resolve(onRejected(reason)) : rejected(reason);
			} catch(e) {
				return rejected(e);
			}
		});

		return p;
	}

	/**
	 * Creates a new, Deferred with fully isolated resolver and promise parts,
	 * either or both of which may be given out safely to consumers.
	 * The Deferred itself has the full API: resolve, reject, progress, and
	 * then. The resolver has resolve, reject, and progress.  The promise
	 * only has then.
	 *
	 * @returns {Deferred}
	 */
	function defer() {
		var deferred, promise, handlers, progressHandlers,
			_then, _progress, _resolve;

		/**
		 * The promise for the new deferred
		 * @type {Promise}
		 */
		promise = new Promise(then);

		/**
		 * The full Deferred object, with {@link Promise} and {@link Resolver} parts
		 * @class Deferred
		 * @name Deferred
		 */
		deferred = {
			then:     then, // DEPRECATED: use deferred.promise.then
			resolve:  promiseResolve,
			reject:   promiseReject,
			// TODO: Consider renaming progress() to notify()
			progress: promiseProgress,

			promise:  promise,

			resolver: {
				resolve:  promiseResolve,
				reject:   promiseReject,
				progress: promiseProgress
			}
		};

		handlers = [];
		progressHandlers = [];

		/**
		 * Pre-resolution then() that adds the supplied callback, errback, and progback
		 * functions to the registered listeners
		 * @private
		 *
		 * @param {function?} [onFulfilled] resolution handler
		 * @param {function?} [onRejected] rejection handler
		 * @param {function?} [onProgress] progress handler
		 */
		_then = function(onFulfilled, onRejected, onProgress) {
			// TODO: Promises/A+ check typeof onFulfilled, onRejected, onProgress
			var deferred, progressHandler;

			deferred = defer();

			progressHandler = typeof onProgress === 'function'
				? function(update) {
					try {
						// Allow progress handler to transform progress event
						deferred.progress(onProgress(update));
					} catch(e) {
						// Use caught value as progress
						deferred.progress(e);
					}
				}
				: function(update) { deferred.progress(update); };

			handlers.push(function(promise) {
				promise.then(onFulfilled, onRejected)
					.then(deferred.resolve, deferred.reject, progressHandler);
			});

			progressHandlers.push(progressHandler);

			return deferred.promise;
		};

		/**
		 * Issue a progress event, notifying all progress listeners
		 * @private
		 * @param {*} update progress event payload to pass to all listeners
		 */
		_progress = function(update) {
			processQueue(progressHandlers, update);
			return update;
		};

		/**
		 * Transition from pre-resolution state to post-resolution state, notifying
		 * all listeners of the resolution or rejection
		 * @private
		 * @param {*} value the value of this deferred
		 */
		_resolve = function(value) {
			value = resolve(value);

			// Replace _then with one that directly notifies with the result.
			_then = value.then;
			// Replace _resolve so that this Deferred can only be resolved once
			_resolve = resolve;
			// Make _progress a noop, to disallow progress for the resolved promise.
			_progress = noop;

			// Notify handlers
			processQueue(handlers, value);

			// Free progressHandlers array since we'll never issue progress events
			progressHandlers = handlers = undef;

			return value;
		};

		return deferred;

		/**
		 * Wrapper to allow _then to be replaced safely
		 * @param {function?} [onFulfilled] resolution handler
		 * @param {function?} [onRejected] rejection handler
		 * @param {function?} [onProgress] progress handler
		 * @returns {Promise} new promise
		 */
		function then(onFulfilled, onRejected, onProgress) {
			// TODO: Promises/A+ check typeof onFulfilled, onRejected, onProgress
			return _then(onFulfilled, onRejected, onProgress);
		}

		/**
		 * Wrapper to allow _resolve to be replaced
		 */
		function promiseResolve(val) {
			return _resolve(val);
		}

		/**
		 * Wrapper to allow _reject to be replaced
		 */
		function promiseReject(err) {
			return _resolve(rejected(err));
		}

		/**
		 * Wrapper to allow _progress to be replaced
		 */
		function promiseProgress(update) {
			return _progress(update);
		}
	}

	/**
	 * Determines if promiseOrValue is a promise or not.  Uses the feature
	 * test from http://wiki.commonjs.org/wiki/Promises/A to determine if
	 * promiseOrValue is a promise.
	 *
	 * @param {*} promiseOrValue anything
	 * @returns {boolean} true if promiseOrValue is a {@link Promise}
	 */
	function isPromise(promiseOrValue) {
		return promiseOrValue && typeof promiseOrValue.then === 'function';
	}

	/**
	 * Initiates a competitive race, returning a promise that will resolve when
	 * howMany of the supplied promisesOrValues have resolved, or will reject when
	 * it becomes impossible for howMany to resolve, for example, when
	 * (promisesOrValues.length - howMany) + 1 input promises reject.
	 *
	 * @param {Array} promisesOrValues array of anything, may contain a mix
	 *      of promises and values
	 * @param howMany {number} number of promisesOrValues to resolve
	 * @param {function?} [onFulfilled] resolution handler
	 * @param {function?} [onRejected] rejection handler
	 * @param {function?} [onProgress] progress handler
	 * @returns {Promise} promise that will resolve to an array of howMany values that
	 * resolved first, or will reject with an array of (promisesOrValues.length - howMany) + 1
	 * rejection reasons.
	 */
	function some(promisesOrValues, howMany, onFulfilled, onRejected, onProgress) {

		checkCallbacks(2, arguments);

		return when(promisesOrValues, function(promisesOrValues) {

			var toResolve, toReject, values, reasons, deferred, fulfillOne, rejectOne, progress, len, i;

			len = promisesOrValues.length >>> 0;

			toResolve = Math.max(0, Math.min(howMany, len));
			values = [];

			toReject = (len - toResolve) + 1;
			reasons = [];

			deferred = defer();

			// No items in the input, resolve immediately
			if (!toResolve) {
				deferred.resolve(values);

			} else {
				progress = deferred.progress;

				rejectOne = function(reason) {
					reasons.push(reason);
					if(!--toReject) {
						fulfillOne = rejectOne = noop;
						deferred.reject(reasons);
					}
				};

				fulfillOne = function(val) {
					// This orders the values based on promise resolution order
					// Another strategy would be to use the original position of
					// the corresponding promise.
					values.push(val);

					if (!--toResolve) {
						fulfillOne = rejectOne = noop;
						deferred.resolve(values);
					}
				};

				for(i = 0; i < len; ++i) {
					if(i in promisesOrValues) {
						when(promisesOrValues[i], fulfiller, rejecter, progress);
					}
				}
			}

			return deferred.then(onFulfilled, onRejected, onProgress);

			function rejecter(reason) {
				rejectOne(reason);
			}

			function fulfiller(val) {
				fulfillOne(val);
			}

		});
	}

	/**
	 * Initiates a competitive race, returning a promise that will resolve when
	 * any one of the supplied promisesOrValues has resolved or will reject when
	 * *all* promisesOrValues have rejected.
	 *
	 * @param {Array|Promise} promisesOrValues array of anything, may contain a mix
	 *      of {@link Promise}s and values
	 * @param {function?} [onFulfilled] resolution handler
	 * @param {function?} [onRejected] rejection handler
	 * @param {function?} [onProgress] progress handler
	 * @returns {Promise} promise that will resolve to the value that resolved first, or
	 * will reject with an array of all rejected inputs.
	 */
	function any(promisesOrValues, onFulfilled, onRejected, onProgress) {

		function unwrapSingleResult(val) {
			return onFulfilled ? onFulfilled(val[0]) : val[0];
		}

		return some(promisesOrValues, 1, unwrapSingleResult, onRejected, onProgress);
	}

	/**
	 * Return a promise that will resolve only once all the supplied promisesOrValues
	 * have resolved. The resolution value of the returned promise will be an array
	 * containing the resolution values of each of the promisesOrValues.
	 * @memberOf when
	 *
	 * @param {Array|Promise} promisesOrValues array of anything, may contain a mix
	 *      of {@link Promise}s and values
	 * @param {function?} [onFulfilled] resolution handler
	 * @param {function?} [onRejected] rejection handler
	 * @param {function?} [onProgress] progress handler
	 * @returns {Promise}
	 */
	function all(promisesOrValues, onFulfilled, onRejected, onProgress) {
		checkCallbacks(1, arguments);
		return map(promisesOrValues, identity).then(onFulfilled, onRejected, onProgress);
	}

	/**
	 * Joins multiple promises into a single returned promise.
	 * @returns {Promise} a promise that will fulfill when *all* the input promises
	 * have fulfilled, or will reject when *any one* of the input promises rejects.
	 */
	function join(/* ...promises */) {
		return map(arguments, identity);
	}

	/**
	 * Traditional map function, similar to `Array.prototype.map()`, but allows
	 * input to contain {@link Promise}s and/or values, and mapFunc may return
	 * either a value or a {@link Promise}
	 *
	 * @param {Array|Promise} promise array of anything, may contain a mix
	 *      of {@link Promise}s and values
	 * @param {function} mapFunc mapping function mapFunc(value) which may return
	 *      either a {@link Promise} or value
	 * @returns {Promise} a {@link Promise} that will resolve to an array containing
	 *      the mapped output values.
	 */
	function map(promise, mapFunc) {
		return when(promise, function(array) {
			var results, len, toResolve, resolve, i, d;

			// Since we know the resulting length, we can preallocate the results
			// array to avoid array expansions.
			toResolve = len = array.length >>> 0;
			results = [];
			d = defer();

			if(!toResolve) {
				d.resolve(results);
			} else {

				resolve = function resolveOne(item, i) {
					when(item, mapFunc).then(function(mapped) {
						results[i] = mapped;

						if(!--toResolve) {
							d.resolve(results);
						}
					}, d.reject);
				};

				// Since mapFunc may be async, get all invocations of it into flight
				for(i = 0; i < len; i++) {
					if(i in array) {
						resolve(array[i], i);
					} else {
						--toResolve;
					}
				}

			}

			return d.promise;

		});
	}

	/**
	 * Traditional reduce function, similar to `Array.prototype.reduce()`, but
	 * input may contain promises and/or values, and reduceFunc
	 * may return either a value or a promise, *and* initialValue may
	 * be a promise for the starting value.
	 *
	 * @param {Array|Promise} promise array or promise for an array of anything,
	 *      may contain a mix of promises and values.
	 * @param {function} reduceFunc reduce function reduce(currentValue, nextValue, index, total),
	 *      where total is the total number of items being reduced, and will be the same
	 *      in each call to reduceFunc.
	 * @returns {Promise} that will resolve to the final reduced value
	 */
	function reduce(promise, reduceFunc /*, initialValue */) {
		var args = slice.call(arguments, 1);

		return when(promise, function(array) {
			var total;

			total = array.length;

			// Wrap the supplied reduceFunc with one that handles promises and then
			// delegates to the supplied.
			args[0] = function (current, val, i) {
				return when(current, function (c) {
					return when(val, function (value) {
						return reduceFunc(c, value, i, total);
					});
				});
			};

			return reduceArray.apply(array, args);
		});
	}

	/**
	 * Ensure that resolution of promiseOrValue will trigger resolver with the
	 * value or reason of promiseOrValue, or instead with resolveValue if it is provided.
	 *
	 * @param promiseOrValue
	 * @param {Object} resolver
	 * @param {function} resolver.resolve
	 * @param {function} resolver.reject
	 * @param {*} [resolveValue]
	 * @returns {Promise}
	 */
	function chain(promiseOrValue, resolver, resolveValue) {
		var useResolveValue = arguments.length > 2;

		return when(promiseOrValue,
			function(val) {
				val = useResolveValue ? resolveValue : val;
				resolver.resolve(val);
				return val;
			},
			function(reason) {
				resolver.reject(reason);
				return rejected(reason);
			},
			resolver.progress
		);
	}

	//
	// Utility functions
	//

	/**
	 * Apply all functions in queue to value
	 * @param {Array} queue array of functions to execute
	 * @param {*} value argument passed to each function
	 */
	function processQueue(queue, value) {
		var handler, i = 0;

		while (handler = queue[i++]) {
			handler(value);
		}
	}

	/**
	 * Helper that checks arrayOfCallbacks to ensure that each element is either
	 * a function, or null or undefined.
	 * @private
	 * @param {number} start index at which to start checking items in arrayOfCallbacks
	 * @param {Array} arrayOfCallbacks array to check
	 * @throws {Error} if any element of arrayOfCallbacks is something other than
	 * a functions, null, or undefined.
	 */
	function checkCallbacks(start, arrayOfCallbacks) {
		// TODO: Promises/A+ update type checking and docs
		var arg, i = arrayOfCallbacks.length;

		while(i > start) {
			arg = arrayOfCallbacks[--i];

			if (arg != null && typeof arg != 'function') {
				throw new Error('arg '+i+' must be a function');
			}
		}
	}

	/**
	 * No-Op function used in method replacement
	 * @private
	 */
	function noop() {}

	slice = [].slice;

	// ES5 reduce implementation if native not available
	// See: http://es5.github.com/#x15.4.4.21 as there are many
	// specifics and edge cases.
	reduceArray = [].reduce ||
		function(reduceFunc /*, initialValue */) {
			/*jshint maxcomplexity: 7*/

			// ES5 dictates that reduce.length === 1

			// This implementation deviates from ES5 spec in the following ways:
			// 1. It does not check if reduceFunc is a Callable

			var arr, args, reduced, len, i;

			i = 0;
			// This generates a jshint warning, despite being valid
			// "Missing 'new' prefix when invoking a constructor."
			// See https://github.com/jshint/jshint/issues/392
			arr = Object(this);
			len = arr.length >>> 0;
			args = arguments;

			// If no initialValue, use first item of array (we know length !== 0 here)
			// and adjust i to start at second item
			if(args.length <= 1) {
				// Skip to the first real element in the array
				for(;;) {
					if(i in arr) {
						reduced = arr[i++];
						break;
					}

					// If we reached the end of the array without finding any real
					// elements, it's a TypeError
					if(++i >= len) {
						throw new TypeError();
					}
				}
			} else {
				// If initialValue provided, use it
				reduced = args[1];
			}

			// Do the actual reduce
			for(;i < len; ++i) {
				// Skip holes
				if(i in arr) {
					reduced = reduceFunc(reduced, arr[i], i, arr);
				}
			}

			return reduced;
		};

	function identity(x) {
		return x;
	}

	return when;
});
})(typeof define == 'function' && define.amd
	? define
	: function (factory) { typeof exports === 'object'
		? (module.exports = factory())
		: (this.when      = factory());
	}
	// Boilerplate for AMD, Node, and browser global
);

define('Core/FeatureDetection',[
        './defaultValue',
        './defined',
        './defineProperties',
        './DeveloperError',
        './Fullscreen',
        '../ThirdParty/when'
    ], function(
        defaultValue,
        defined,
        defineProperties,
        DeveloperError,
        Fullscreen,
        when) {
    'use strict';
    /*global CanvasPixelArray*/

    var theNavigator;
    if (typeof navigator !== 'undefined') {
        theNavigator = navigator;
    } else {
        theNavigator = {};
    }

    function extractVersion(versionString) {
        var parts = versionString.split('.');
        for (var i = 0, len = parts.length; i < len; ++i) {
            parts[i] = parseInt(parts[i], 10);
        }
        return parts;
    }

    var isChromeResult;
    var chromeVersionResult;
    function isChrome() {
        if (!defined(isChromeResult)) {
            isChromeResult = false;
            // Edge contains Chrome in the user agent too
            if (!isEdge()) {
                var fields = (/ Chrome\/([\.0-9]+)/).exec(theNavigator.userAgent);
                if (fields !== null) {
                    isChromeResult = true;
                    chromeVersionResult = extractVersion(fields[1]);
                }
            }
        }

        return isChromeResult;
    }

    function chromeVersion() {
        return isChrome() && chromeVersionResult;
    }

    var isSafariResult;
    var safariVersionResult;
    function isSafari() {
        if (!defined(isSafariResult)) {
            isSafariResult = false;

            // Chrome and Edge contain Safari in the user agent too
            if (!isChrome() && !isEdge() && (/ Safari\/[\.0-9]+/).test(theNavigator.userAgent)) {
                var fields = (/ Version\/([\.0-9]+)/).exec(theNavigator.userAgent);
                if (fields !== null) {
                    isSafariResult = true;
                    safariVersionResult = extractVersion(fields[1]);
                }
            }
        }

        return isSafariResult;
    }

    function safariVersion() {
        return isSafari() && safariVersionResult;
    }

    var isWebkitResult;
    var webkitVersionResult;
    function isWebkit() {
        if (!defined(isWebkitResult)) {
            isWebkitResult = false;

            var fields = (/ AppleWebKit\/([\.0-9]+)(\+?)/).exec(theNavigator.userAgent);
            if (fields !== null) {
                isWebkitResult = true;
                webkitVersionResult = extractVersion(fields[1]);
                webkitVersionResult.isNightly = !!fields[2];
            }
        }

        return isWebkitResult;
    }

    function webkitVersion() {
        return isWebkit() && webkitVersionResult;
    }

    var isInternetExplorerResult;
    var internetExplorerVersionResult;
    function isInternetExplorer() {
        if (!defined(isInternetExplorerResult)) {
            isInternetExplorerResult = false;

            var fields;
            if (theNavigator.appName === 'Microsoft Internet Explorer') {
                fields = /MSIE ([0-9]{1,}[\.0-9]{0,})/.exec(theNavigator.userAgent);
                if (fields !== null) {
                    isInternetExplorerResult = true;
                    internetExplorerVersionResult = extractVersion(fields[1]);
                }
            } else if (theNavigator.appName === 'Netscape') {
                fields = /Trident\/.*rv:([0-9]{1,}[\.0-9]{0,})/.exec(theNavigator.userAgent);
                if (fields !== null) {
                    isInternetExplorerResult = true;
                    internetExplorerVersionResult = extractVersion(fields[1]);
                }
            }
        }
        return isInternetExplorerResult;
    }

    function internetExplorerVersion() {
        return isInternetExplorer() && internetExplorerVersionResult;
    }

    var isEdgeResult;
    var edgeVersionResult;
    function isEdge() {
        if (!defined(isEdgeResult)) {
            isEdgeResult = false;
            var fields = (/ Edge\/([\.0-9]+)/).exec(theNavigator.userAgent);
            if (fields !== null) {
                isEdgeResult = true;
                edgeVersionResult = extractVersion(fields[1]);
            }
        }
        return isEdgeResult;
    }

    function edgeVersion() {
        return isEdge() && edgeVersionResult;
    }

    var isFirefoxResult;
    var firefoxVersionResult;
    function isFirefox() {
        if (!defined(isFirefoxResult)) {
            isFirefoxResult = false;

            var fields = /Firefox\/([\.0-9]+)/.exec(theNavigator.userAgent);
            if (fields !== null) {
                isFirefoxResult = true;
                firefoxVersionResult = extractVersion(fields[1]);
            }
        }
        return isFirefoxResult;
    }

    var isWindowsResult;
    function isWindows() {
        if (!defined(isWindowsResult)) {
            isWindowsResult = /Windows/i.test(theNavigator.appVersion);
        }
        return isWindowsResult;
    }

    function firefoxVersion() {
        return isFirefox() && firefoxVersionResult;
    }

    var hasPointerEvents;
    function supportsPointerEvents() {
        if (!defined(hasPointerEvents)) {
            //While navigator.pointerEnabled is deprecated in the W3C specification
            //we still need to use it if it exists in order to support browsers
            //that rely on it, such as the Windows WebBrowser control which defines
            //PointerEvent but sets navigator.pointerEnabled to false.

            //Firefox disabled because of https://github.com/AnalyticalGraphicsInc/cesium/issues/6372
            hasPointerEvents = !isFirefox() && typeof PointerEvent !== 'undefined' && (!defined(theNavigator.pointerEnabled) || theNavigator.pointerEnabled);
        }
        return hasPointerEvents;
    }

    var imageRenderingValueResult;
    var supportsImageRenderingPixelatedResult;
    function supportsImageRenderingPixelated() {
        if (!defined(supportsImageRenderingPixelatedResult)) {
            var canvas = document.createElement('canvas');
            canvas.setAttribute('style',
                                'image-rendering: -moz-crisp-edges;' +
                                'image-rendering: pixelated;');
            //canvas.style.imageRendering will be undefined, null or an empty string on unsupported browsers.
            var tmp = canvas.style.imageRendering;
            supportsImageRenderingPixelatedResult = defined(tmp) && tmp !== '';
            if (supportsImageRenderingPixelatedResult) {
                imageRenderingValueResult = tmp;
            }
        }
        return supportsImageRenderingPixelatedResult;
    }

    function imageRenderingValue() {
        return supportsImageRenderingPixelated() ? imageRenderingValueResult : undefined;
    }

    function supportsWebP() {
                if (!supportsWebP.initialized) {
            throw new DeveloperError('You must call FeatureDetection.supportsWebP.initialize and wait for the promise to resolve before calling FeatureDetection.supportsWebP');
        }
                return supportsWebP._result;
    }
    supportsWebP._promise = undefined;
    supportsWebP._result = undefined;
    supportsWebP.initialize = function() {
        // From https://developers.google.com/speed/webp/faq#how_can_i_detect_browser_support_for_webp
        if (defined(supportsWebP._promise)) {
            return supportsWebP._promise;
        }

        var supportsWebPDeferred = when.defer();
        supportsWebP._promise = supportsWebPDeferred.promise;
        if (isEdge()) {
            // Edge's WebP support with WebGL is incomplete.
            // See bug report: https://developer.microsoft.com/en-us/microsoft-edge/platform/issues/19221241/
            supportsWebP._result = false;
            supportsWebPDeferred.resolve(supportsWebP._result);
            return supportsWebPDeferred.promise;
        }

        var image = new Image();
        image.onload = function () {
            supportsWebP._result = (image.width > 0) && (image.height > 0);
            supportsWebPDeferred.resolve(supportsWebP._result);
        };

        image.onerror = function () {
            supportsWebP._result = false;
            supportsWebPDeferred.resolve(supportsWebP._result);
        };

        image.src = 'data:image/webp;base64,UklGRiIAAABXRUJQVlA4IBYAAAAwAQCdASoBAAEADsD+JaQAA3AAAAAA';

        return supportsWebPDeferred.promise;
    };
    defineProperties(supportsWebP, {
        initialized: {
            get: function() {
                return defined(supportsWebP._result);
            }
        }
    });

    var typedArrayTypes = [];
    if (typeof ArrayBuffer !== 'undefined') {
        typedArrayTypes.push(Int8Array, Uint8Array, Int16Array, Uint16Array, Int32Array, Uint32Array, Float32Array, Float64Array);

        if (typeof Uint8ClampedArray !== 'undefined') {
            typedArrayTypes.push(Uint8ClampedArray);
        }

        if (typeof CanvasPixelArray !== 'undefined') {
            typedArrayTypes.push(CanvasPixelArray);
        }
    }

    /**
     * A set of functions to detect whether the current browser supports
     * various features.
     *
     * @exports FeatureDetection
     */
    var FeatureDetection = {
        isChrome : isChrome,
        chromeVersion : chromeVersion,
        isSafari : isSafari,
        safariVersion : safariVersion,
        isWebkit : isWebkit,
        webkitVersion : webkitVersion,
        isInternetExplorer : isInternetExplorer,
        internetExplorerVersion : internetExplorerVersion,
        isEdge : isEdge,
        edgeVersion : edgeVersion,
        isFirefox : isFirefox,
        firefoxVersion : firefoxVersion,
        isWindows : isWindows,
        hardwareConcurrency : defaultValue(theNavigator.hardwareConcurrency, 3),
        supportsPointerEvents : supportsPointerEvents,
        supportsImageRenderingPixelated: supportsImageRenderingPixelated,
        supportsWebP: supportsWebP,
        imageRenderingValue: imageRenderingValue,
        typedArrayTypes: typedArrayTypes
    };

    /**
     * Detects whether the current browser supports the full screen standard.
     *
     * @returns {Boolean} true if the browser supports the full screen standard, false if not.
     *
     * @see Fullscreen
     * @see {@link http://dvcs.w3.org/hg/fullscreen/raw-file/tip/Overview.html|W3C Fullscreen Living Specification}
     */
    FeatureDetection.supportsFullscreen = function() {
        return Fullscreen.supportsFullscreen();
    };

    /**
     * Detects whether the current browser supports typed arrays.
     *
     * @returns {Boolean} true if the browser supports typed arrays, false if not.
     *
     * @see {@link http://www.khronos.org/registry/typedarray/specs/latest/|Typed Array Specification}
     */
    FeatureDetection.supportsTypedArrays = function() {
        return typeof ArrayBuffer !== 'undefined';
    };

    /**
     * Detects whether the current browser supports Web Workers.
     *
     * @returns {Boolean} true if the browsers supports Web Workers, false if not.
     *
     * @see {@link http://www.w3.org/TR/workers/}
     */
    FeatureDetection.supportsWebWorkers = function() {
        return typeof Worker !== 'undefined';
    };

    /**
     * Detects whether the current browser supports Web Assembly.
     *
     * @returns {Boolean} true if the browsers supports Web Assembly, false if not.
     *
     * @see {@link https://developer.mozilla.org/en-US/docs/WebAssembly}
     */
    FeatureDetection.supportsWebAssembly = function() {
        return typeof WebAssembly !== 'undefined' && !FeatureDetection.isEdge();
    };

    return FeatureDetection;
});

define('Core/WebGLConstants',[
        './freezeObject'
    ], function(
        freezeObject) {
    'use strict';

    /**
     * Enum containing WebGL Constant values by name.
     * for use without an active WebGL context, or in cases where certain constants are unavailable using the WebGL context
     * (For example, in [Safari 9]{@link https://github.com/AnalyticalGraphicsInc/cesium/issues/2989}).
     *
     * These match the constants from the [WebGL 1.0]{@link https://www.khronos.org/registry/webgl/specs/latest/1.0/}
     * and [WebGL 2.0]{@link https://www.khronos.org/registry/webgl/specs/latest/2.0/}
     * specifications.
     *
     * @exports WebGLConstants
     */
    var WebGLConstants = {
        DEPTH_BUFFER_BIT : 0x00000100,
        STENCIL_BUFFER_BIT : 0x00000400,
        COLOR_BUFFER_BIT : 0x00004000,
        POINTS : 0x0000,
        LINES : 0x0001,
        LINE_LOOP : 0x0002,
        LINE_STRIP : 0x0003,
        TRIANGLES : 0x0004,
        TRIANGLE_STRIP : 0x0005,
        TRIANGLE_FAN : 0x0006,
        ZERO : 0,
        ONE : 1,
        SRC_COLOR : 0x0300,
        ONE_MINUS_SRC_COLOR : 0x0301,
        SRC_ALPHA : 0x0302,
        ONE_MINUS_SRC_ALPHA : 0x0303,
        DST_ALPHA : 0x0304,
        ONE_MINUS_DST_ALPHA : 0x0305,
        DST_COLOR : 0x0306,
        ONE_MINUS_DST_COLOR : 0x0307,
        SRC_ALPHA_SATURATE : 0x0308,
        FUNC_ADD : 0x8006,
        BLEND_EQUATION : 0x8009,
        BLEND_EQUATION_RGB : 0x8009, // same as BLEND_EQUATION
        BLEND_EQUATION_ALPHA : 0x883D,
        FUNC_SUBTRACT : 0x800A,
        FUNC_REVERSE_SUBTRACT : 0x800B,
        BLEND_DST_RGB : 0x80C8,
        BLEND_SRC_RGB : 0x80C9,
        BLEND_DST_ALPHA : 0x80CA,
        BLEND_SRC_ALPHA : 0x80CB,
        CONSTANT_COLOR : 0x8001,
        ONE_MINUS_CONSTANT_COLOR : 0x8002,
        CONSTANT_ALPHA : 0x8003,
        ONE_MINUS_CONSTANT_ALPHA : 0x8004,
        BLEND_COLOR : 0x8005,
        ARRAY_BUFFER : 0x8892,
        ELEMENT_ARRAY_BUFFER : 0x8893,
        ARRAY_BUFFER_BINDING : 0x8894,
        ELEMENT_ARRAY_BUFFER_BINDING : 0x8895,
        STREAM_DRAW : 0x88E0,
        STATIC_DRAW : 0x88E4,
        DYNAMIC_DRAW : 0x88E8,
        BUFFER_SIZE : 0x8764,
        BUFFER_USAGE : 0x8765,
        CURRENT_VERTEX_ATTRIB : 0x8626,
        FRONT : 0x0404,
        BACK : 0x0405,
        FRONT_AND_BACK : 0x0408,
        CULL_FACE : 0x0B44,
        BLEND : 0x0BE2,
        DITHER : 0x0BD0,
        STENCIL_TEST : 0x0B90,
        DEPTH_TEST : 0x0B71,
        SCISSOR_TEST : 0x0C11,
        POLYGON_OFFSET_FILL : 0x8037,
        SAMPLE_ALPHA_TO_COVERAGE : 0x809E,
        SAMPLE_COVERAGE : 0x80A0,
        NO_ERROR : 0,
        INVALID_ENUM : 0x0500,
        INVALID_VALUE : 0x0501,
        INVALID_OPERATION : 0x0502,
        OUT_OF_MEMORY : 0x0505,
        CW : 0x0900,
        CCW : 0x0901,
        LINE_WIDTH : 0x0B21,
        ALIASED_POINT_SIZE_RANGE : 0x846D,
        ALIASED_LINE_WIDTH_RANGE : 0x846E,
        CULL_FACE_MODE : 0x0B45,
        FRONT_FACE : 0x0B46,
        DEPTH_RANGE : 0x0B70,
        DEPTH_WRITEMASK : 0x0B72,
        DEPTH_CLEAR_VALUE : 0x0B73,
        DEPTH_FUNC : 0x0B74,
        STENCIL_CLEAR_VALUE : 0x0B91,
        STENCIL_FUNC : 0x0B92,
        STENCIL_FAIL : 0x0B94,
        STENCIL_PASS_DEPTH_FAIL : 0x0B95,
        STENCIL_PASS_DEPTH_PASS : 0x0B96,
        STENCIL_REF : 0x0B97,
        STENCIL_VALUE_MASK : 0x0B93,
        STENCIL_WRITEMASK : 0x0B98,
        STENCIL_BACK_FUNC : 0x8800,
        STENCIL_BACK_FAIL : 0x8801,
        STENCIL_BACK_PASS_DEPTH_FAIL : 0x8802,
        STENCIL_BACK_PASS_DEPTH_PASS : 0x8803,
        STENCIL_BACK_REF : 0x8CA3,
        STENCIL_BACK_VALUE_MASK : 0x8CA4,
        STENCIL_BACK_WRITEMASK : 0x8CA5,
        VIEWPORT : 0x0BA2,
        SCISSOR_BOX : 0x0C10,
        COLOR_CLEAR_VALUE : 0x0C22,
        COLOR_WRITEMASK : 0x0C23,
        UNPACK_ALIGNMENT : 0x0CF5,
        PACK_ALIGNMENT : 0x0D05,
        MAX_TEXTURE_SIZE : 0x0D33,
        MAX_VIEWPORT_DIMS : 0x0D3A,
        SUBPIXEL_BITS : 0x0D50,
        RED_BITS : 0x0D52,
        GREEN_BITS : 0x0D53,
        BLUE_BITS : 0x0D54,
        ALPHA_BITS : 0x0D55,
        DEPTH_BITS : 0x0D56,
        STENCIL_BITS : 0x0D57,
        POLYGON_OFFSET_UNITS : 0x2A00,
        POLYGON_OFFSET_FACTOR : 0x8038,
        TEXTURE_BINDING_2D : 0x8069,
        SAMPLE_BUFFERS : 0x80A8,
        SAMPLES : 0x80A9,
        SAMPLE_COVERAGE_VALUE : 0x80AA,
        SAMPLE_COVERAGE_INVERT : 0x80AB,
        COMPRESSED_TEXTURE_FORMATS : 0x86A3,
        DONT_CARE : 0x1100,
        FASTEST : 0x1101,
        NICEST : 0x1102,
        GENERATE_MIPMAP_HINT : 0x8192,
        BYTE : 0x1400,
        UNSIGNED_BYTE : 0x1401,
        SHORT : 0x1402,
        UNSIGNED_SHORT : 0x1403,
        INT : 0x1404,
        UNSIGNED_INT : 0x1405,
        FLOAT : 0x1406,
        DEPTH_COMPONENT : 0x1902,
        ALPHA : 0x1906,
        RGB : 0x1907,
        RGBA : 0x1908,
        LUMINANCE : 0x1909,
        LUMINANCE_ALPHA : 0x190A,
        UNSIGNED_SHORT_4_4_4_4 : 0x8033,
        UNSIGNED_SHORT_5_5_5_1 : 0x8034,
        UNSIGNED_SHORT_5_6_5 : 0x8363,
        FRAGMENT_SHADER : 0x8B30,
        VERTEX_SHADER : 0x8B31,
        MAX_VERTEX_ATTRIBS : 0x8869,
        MAX_VERTEX_UNIFORM_VECTORS : 0x8DFB,
        MAX_VARYING_VECTORS : 0x8DFC,
        MAX_COMBINED_TEXTURE_IMAGE_UNITS : 0x8B4D,
        MAX_VERTEX_TEXTURE_IMAGE_UNITS : 0x8B4C,
        MAX_TEXTURE_IMAGE_UNITS : 0x8872,
        MAX_FRAGMENT_UNIFORM_VECTORS : 0x8DFD,
        SHADER_TYPE : 0x8B4F,
        DELETE_STATUS : 0x8B80,
        LINK_STATUS : 0x8B82,
        VALIDATE_STATUS : 0x8B83,
        ATTACHED_SHADERS : 0x8B85,
        ACTIVE_UNIFORMS : 0x8B86,
        ACTIVE_ATTRIBUTES : 0x8B89,
        SHADING_LANGUAGE_VERSION : 0x8B8C,
        CURRENT_PROGRAM : 0x8B8D,
        NEVER : 0x0200,
        LESS : 0x0201,
        EQUAL : 0x0202,
        LEQUAL : 0x0203,
        GREATER : 0x0204,
        NOTEQUAL : 0x0205,
        GEQUAL : 0x0206,
        ALWAYS : 0x0207,
        KEEP : 0x1E00,
        REPLACE : 0x1E01,
        INCR : 0x1E02,
        DECR : 0x1E03,
        INVERT : 0x150A,
        INCR_WRAP : 0x8507,
        DECR_WRAP : 0x8508,
        VENDOR : 0x1F00,
        RENDERER : 0x1F01,
        VERSION : 0x1F02,
        NEAREST : 0x2600,
        LINEAR : 0x2601,
        NEAREST_MIPMAP_NEAREST : 0x2700,
        LINEAR_MIPMAP_NEAREST : 0x2701,
        NEAREST_MIPMAP_LINEAR : 0x2702,
        LINEAR_MIPMAP_LINEAR : 0x2703,
        TEXTURE_MAG_FILTER : 0x2800,
        TEXTURE_MIN_FILTER : 0x2801,
        TEXTURE_WRAP_S : 0x2802,
        TEXTURE_WRAP_T : 0x2803,
        TEXTURE_2D : 0x0DE1,
        TEXTURE : 0x1702,
        TEXTURE_CUBE_MAP : 0x8513,
        TEXTURE_BINDING_CUBE_MAP : 0x8514,
        TEXTURE_CUBE_MAP_POSITIVE_X : 0x8515,
        TEXTURE_CUBE_MAP_NEGATIVE_X : 0x8516,
        TEXTURE_CUBE_MAP_POSITIVE_Y : 0x8517,
        TEXTURE_CUBE_MAP_NEGATIVE_Y : 0x8518,
        TEXTURE_CUBE_MAP_POSITIVE_Z : 0x8519,
        TEXTURE_CUBE_MAP_NEGATIVE_Z : 0x851A,
        MAX_CUBE_MAP_TEXTURE_SIZE : 0x851C,
        TEXTURE0 : 0x84C0,
        TEXTURE1 : 0x84C1,
        TEXTURE2 : 0x84C2,
        TEXTURE3 : 0x84C3,
        TEXTURE4 : 0x84C4,
        TEXTURE5 : 0x84C5,
        TEXTURE6 : 0x84C6,
        TEXTURE7 : 0x84C7,
        TEXTURE8 : 0x84C8,
        TEXTURE9 : 0x84C9,
        TEXTURE10 : 0x84CA,
        TEXTURE11 : 0x84CB,
        TEXTURE12 : 0x84CC,
        TEXTURE13 : 0x84CD,
        TEXTURE14 : 0x84CE,
        TEXTURE15 : 0x84CF,
        TEXTURE16 : 0x84D0,
        TEXTURE17 : 0x84D1,
        TEXTURE18 : 0x84D2,
        TEXTURE19 : 0x84D3,
        TEXTURE20 : 0x84D4,
        TEXTURE21 : 0x84D5,
        TEXTURE22 : 0x84D6,
        TEXTURE23 : 0x84D7,
        TEXTURE24 : 0x84D8,
        TEXTURE25 : 0x84D9,
        TEXTURE26 : 0x84DA,
        TEXTURE27 : 0x84DB,
        TEXTURE28 : 0x84DC,
        TEXTURE29 : 0x84DD,
        TEXTURE30 : 0x84DE,
        TEXTURE31 : 0x84DF,
        ACTIVE_TEXTURE : 0x84E0,
        REPEAT : 0x2901,
        CLAMP_TO_EDGE : 0x812F,
        MIRRORED_REPEAT : 0x8370,
        FLOAT_VEC2 : 0x8B50,
        FLOAT_VEC3 : 0x8B51,
        FLOAT_VEC4 : 0x8B52,
        INT_VEC2 : 0x8B53,
        INT_VEC3 : 0x8B54,
        INT_VEC4 : 0x8B55,
        BOOL : 0x8B56,
        BOOL_VEC2 : 0x8B57,
        BOOL_VEC3 : 0x8B58,
        BOOL_VEC4 : 0x8B59,
        FLOAT_MAT2 : 0x8B5A,
        FLOAT_MAT3 : 0x8B5B,
        FLOAT_MAT4 : 0x8B5C,
        SAMPLER_2D : 0x8B5E,
        SAMPLER_CUBE : 0x8B60,
        VERTEX_ATTRIB_ARRAY_ENABLED : 0x8622,
        VERTEX_ATTRIB_ARRAY_SIZE : 0x8623,
        VERTEX_ATTRIB_ARRAY_STRIDE : 0x8624,
        VERTEX_ATTRIB_ARRAY_TYPE : 0x8625,
        VERTEX_ATTRIB_ARRAY_NORMALIZED : 0x886A,
        VERTEX_ATTRIB_ARRAY_POINTER : 0x8645,
        VERTEX_ATTRIB_ARRAY_BUFFER_BINDING : 0x889F,
        IMPLEMENTATION_COLOR_READ_TYPE : 0x8B9A,
        IMPLEMENTATION_COLOR_READ_FORMAT : 0x8B9B,
        COMPILE_STATUS : 0x8B81,
        LOW_FLOAT : 0x8DF0,
        MEDIUM_FLOAT : 0x8DF1,
        HIGH_FLOAT : 0x8DF2,
        LOW_INT : 0x8DF3,
        MEDIUM_INT : 0x8DF4,
        HIGH_INT : 0x8DF5,
        FRAMEBUFFER : 0x8D40,
        RENDERBUFFER : 0x8D41,
        RGBA4 : 0x8056,
        RGB5_A1 : 0x8057,
        RGB565 : 0x8D62,
        DEPTH_COMPONENT16 : 0x81A5,
        STENCIL_INDEX : 0x1901,
        STENCIL_INDEX8 : 0x8D48,
        DEPTH_STENCIL : 0x84F9,
        RENDERBUFFER_WIDTH : 0x8D42,
        RENDERBUFFER_HEIGHT : 0x8D43,
        RENDERBUFFER_INTERNAL_FORMAT : 0x8D44,
        RENDERBUFFER_RED_SIZE : 0x8D50,
        RENDERBUFFER_GREEN_SIZE : 0x8D51,
        RENDERBUFFER_BLUE_SIZE : 0x8D52,
        RENDERBUFFER_ALPHA_SIZE : 0x8D53,
        RENDERBUFFER_DEPTH_SIZE : 0x8D54,
        RENDERBUFFER_STENCIL_SIZE : 0x8D55,
        FRAMEBUFFER_ATTACHMENT_OBJECT_TYPE : 0x8CD0,
        FRAMEBUFFER_ATTACHMENT_OBJECT_NAME : 0x8CD1,
        FRAMEBUFFER_ATTACHMENT_TEXTURE_LEVEL : 0x8CD2,
        FRAMEBUFFER_ATTACHMENT_TEXTURE_CUBE_MAP_FACE : 0x8CD3,
        COLOR_ATTACHMENT0 : 0x8CE0,
        DEPTH_ATTACHMENT : 0x8D00,
        STENCIL_ATTACHMENT : 0x8D20,
        DEPTH_STENCIL_ATTACHMENT : 0x821A,
        NONE : 0,
        FRAMEBUFFER_COMPLETE : 0x8CD5,
        FRAMEBUFFER_INCOMPLETE_ATTACHMENT : 0x8CD6,
        FRAMEBUFFER_INCOMPLETE_MISSING_ATTACHMENT : 0x8CD7,
        FRAMEBUFFER_INCOMPLETE_DIMENSIONS : 0x8CD9,
        FRAMEBUFFER_UNSUPPORTED : 0x8CDD,
        FRAMEBUFFER_BINDING : 0x8CA6,
        RENDERBUFFER_BINDING : 0x8CA7,
        MAX_RENDERBUFFER_SIZE : 0x84E8,
        INVALID_FRAMEBUFFER_OPERATION : 0x0506,
        UNPACK_FLIP_Y_WEBGL : 0x9240,
        UNPACK_PREMULTIPLY_ALPHA_WEBGL : 0x9241,
        CONTEXT_LOST_WEBGL : 0x9242,
        UNPACK_COLORSPACE_CONVERSION_WEBGL : 0x9243,
        BROWSER_DEFAULT_WEBGL : 0x9244,

        // WEBGL_compressed_texture_s3tc
        COMPRESSED_RGB_S3TC_DXT1_EXT : 0x83F0,
        COMPRESSED_RGBA_S3TC_DXT1_EXT : 0x83F1,
        COMPRESSED_RGBA_S3TC_DXT3_EXT : 0x83F2,
        COMPRESSED_RGBA_S3TC_DXT5_EXT : 0x83F3,

        // WEBGL_compressed_texture_pvrtc
        COMPRESSED_RGB_PVRTC_4BPPV1_IMG : 0x8C00,
        COMPRESSED_RGB_PVRTC_2BPPV1_IMG : 0x8C01,
        COMPRESSED_RGBA_PVRTC_4BPPV1_IMG : 0x8C02,
        COMPRESSED_RGBA_PVRTC_2BPPV1_IMG : 0x8C03,

        // WEBGL_compressed_texture_etc1
        COMPRESSED_RGB_ETC1_WEBGL : 0x8D64,

        // EXT_color_buffer_half_float
        HALF_FLOAT_OES : 0x8D61,

        // Desktop OpenGL
        DOUBLE : 0x140A,

        // WebGL 2
        READ_BUFFER : 0x0C02,
        UNPACK_ROW_LENGTH : 0x0CF2,
        UNPACK_SKIP_ROWS : 0x0CF3,
        UNPACK_SKIP_PIXELS : 0x0CF4,
        PACK_ROW_LENGTH : 0x0D02,
        PACK_SKIP_ROWS : 0x0D03,
        PACK_SKIP_PIXELS : 0x0D04,
        COLOR : 0x1800,
        DEPTH : 0x1801,
        STENCIL : 0x1802,
        RED : 0x1903,
        RGB8 : 0x8051,
        RGBA8 : 0x8058,
        RGB10_A2 : 0x8059,
        TEXTURE_BINDING_3D : 0x806A,
        UNPACK_SKIP_IMAGES : 0x806D,
        UNPACK_IMAGE_HEIGHT : 0x806E,
        TEXTURE_3D : 0x806F,
        TEXTURE_WRAP_R : 0x8072,
        MAX_3D_TEXTURE_SIZE : 0x8073,
        UNSIGNED_INT_2_10_10_10_REV : 0x8368,
        MAX_ELEMENTS_VERTICES : 0x80E8,
        MAX_ELEMENTS_INDICES : 0x80E9,
        TEXTURE_MIN_LOD : 0x813A,
        TEXTURE_MAX_LOD : 0x813B,
        TEXTURE_BASE_LEVEL : 0x813C,
        TEXTURE_MAX_LEVEL : 0x813D,
        MIN : 0x8007,
        MAX : 0x8008,
        DEPTH_COMPONENT24 : 0x81A6,
        MAX_TEXTURE_LOD_BIAS : 0x84FD,
        TEXTURE_COMPARE_MODE : 0x884C,
        TEXTURE_COMPARE_FUNC : 0x884D,
        CURRENT_QUERY : 0x8865,
        QUERY_RESULT : 0x8866,
        QUERY_RESULT_AVAILABLE : 0x8867,
        STREAM_READ : 0x88E1,
        STREAM_COPY : 0x88E2,
        STATIC_READ : 0x88E5,
        STATIC_COPY : 0x88E6,
        DYNAMIC_READ : 0x88E9,
        DYNAMIC_COPY : 0x88EA,
        MAX_DRAW_BUFFERS : 0x8824,
        DRAW_BUFFER0 : 0x8825,
        DRAW_BUFFER1 : 0x8826,
        DRAW_BUFFER2 : 0x8827,
        DRAW_BUFFER3 : 0x8828,
        DRAW_BUFFER4 : 0x8829,
        DRAW_BUFFER5 : 0x882A,
        DRAW_BUFFER6 : 0x882B,
        DRAW_BUFFER7 : 0x882C,
        DRAW_BUFFER8 : 0x882D,
        DRAW_BUFFER9 : 0x882E,
        DRAW_BUFFER10 : 0x882F,
        DRAW_BUFFER11 : 0x8830,
        DRAW_BUFFER12 : 0x8831,
        DRAW_BUFFER13 : 0x8832,
        DRAW_BUFFER14 : 0x8833,
        DRAW_BUFFER15 : 0x8834,
        MAX_FRAGMENT_UNIFORM_COMPONENTS : 0x8B49,
        MAX_VERTEX_UNIFORM_COMPONENTS : 0x8B4A,
        SAMPLER_3D : 0x8B5F,
        SAMPLER_2D_SHADOW : 0x8B62,
        FRAGMENT_SHADER_DERIVATIVE_HINT : 0x8B8B,
        PIXEL_PACK_BUFFER : 0x88EB,
        PIXEL_UNPACK_BUFFER : 0x88EC,
        PIXEL_PACK_BUFFER_BINDING : 0x88ED,
        PIXEL_UNPACK_BUFFER_BINDING : 0x88EF,
        FLOAT_MAT2x3 : 0x8B65,
        FLOAT_MAT2x4 : 0x8B66,
        FLOAT_MAT3x2 : 0x8B67,
        FLOAT_MAT3x4 : 0x8B68,
        FLOAT_MAT4x2 : 0x8B69,
        FLOAT_MAT4x3 : 0x8B6A,
        SRGB : 0x8C40,
        SRGB8 : 0x8C41,
        SRGB8_ALPHA8 : 0x8C43,
        COMPARE_REF_TO_TEXTURE : 0x884E,
        RGBA32F : 0x8814,
        RGB32F : 0x8815,
        RGBA16F : 0x881A,
        RGB16F : 0x881B,
        VERTEX_ATTRIB_ARRAY_INTEGER : 0x88FD,
        MAX_ARRAY_TEXTURE_LAYERS : 0x88FF,
        MIN_PROGRAM_TEXEL_OFFSET : 0x8904,
        MAX_PROGRAM_TEXEL_OFFSET : 0x8905,
        MAX_VARYING_COMPONENTS : 0x8B4B,
        TEXTURE_2D_ARRAY : 0x8C1A,
        TEXTURE_BINDING_2D_ARRAY : 0x8C1D,
        R11F_G11F_B10F : 0x8C3A,
        UNSIGNED_INT_10F_11F_11F_REV : 0x8C3B,
        RGB9_E5 : 0x8C3D,
        UNSIGNED_INT_5_9_9_9_REV : 0x8C3E,
        TRANSFORM_FEEDBACK_BUFFER_MODE : 0x8C7F,
        MAX_TRANSFORM_FEEDBACK_SEPARATE_COMPONENTS : 0x8C80,
        TRANSFORM_FEEDBACK_VARYINGS : 0x8C83,
        TRANSFORM_FEEDBACK_BUFFER_START : 0x8C84,
        TRANSFORM_FEEDBACK_BUFFER_SIZE : 0x8C85,
        TRANSFORM_FEEDBACK_PRIMITIVES_WRITTEN : 0x8C88,
        RASTERIZER_DISCARD : 0x8C89,
        MAX_TRANSFORM_FEEDBACK_INTERLEAVED_COMPONENTS : 0x8C8A,
        MAX_TRANSFORM_FEEDBACK_SEPARATE_ATTRIBS : 0x8C8B,
        INTERLEAVED_ATTRIBS : 0x8C8C,
        SEPARATE_ATTRIBS : 0x8C8D,
        TRANSFORM_FEEDBACK_BUFFER : 0x8C8E,
        TRANSFORM_FEEDBACK_BUFFER_BINDING : 0x8C8F,
        RGBA32UI : 0x8D70,
        RGB32UI : 0x8D71,
        RGBA16UI : 0x8D76,
        RGB16UI : 0x8D77,
        RGBA8UI : 0x8D7C,
        RGB8UI : 0x8D7D,
        RGBA32I : 0x8D82,
        RGB32I : 0x8D83,
        RGBA16I : 0x8D88,
        RGB16I : 0x8D89,
        RGBA8I : 0x8D8E,
        RGB8I : 0x8D8F,
        RED_INTEGER : 0x8D94,
        RGB_INTEGER : 0x8D98,
        RGBA_INTEGER : 0x8D99,
        SAMPLER_2D_ARRAY : 0x8DC1,
        SAMPLER_2D_ARRAY_SHADOW : 0x8DC4,
        SAMPLER_CUBE_SHADOW : 0x8DC5,
        UNSIGNED_INT_VEC2 : 0x8DC6,
        UNSIGNED_INT_VEC3 : 0x8DC7,
        UNSIGNED_INT_VEC4 : 0x8DC8,
        INT_SAMPLER_2D : 0x8DCA,
        INT_SAMPLER_3D : 0x8DCB,
        INT_SAMPLER_CUBE : 0x8DCC,
        INT_SAMPLER_2D_ARRAY : 0x8DCF,
        UNSIGNED_INT_SAMPLER_2D : 0x8DD2,
        UNSIGNED_INT_SAMPLER_3D : 0x8DD3,
        UNSIGNED_INT_SAMPLER_CUBE : 0x8DD4,
        UNSIGNED_INT_SAMPLER_2D_ARRAY : 0x8DD7,
        DEPTH_COMPONENT32F : 0x8CAC,
        DEPTH32F_STENCIL8 : 0x8CAD,
        FLOAT_32_UNSIGNED_INT_24_8_REV : 0x8DAD,
        FRAMEBUFFER_ATTACHMENT_COLOR_ENCODING : 0x8210,
        FRAMEBUFFER_ATTACHMENT_COMPONENT_TYPE : 0x8211,
        FRAMEBUFFER_ATTACHMENT_RED_SIZE : 0x8212,
        FRAMEBUFFER_ATTACHMENT_GREEN_SIZE : 0x8213,
        FRAMEBUFFER_ATTACHMENT_BLUE_SIZE : 0x8214,
        FRAMEBUFFER_ATTACHMENT_ALPHA_SIZE : 0x8215,
        FRAMEBUFFER_ATTACHMENT_DEPTH_SIZE : 0x8216,
        FRAMEBUFFER_ATTACHMENT_STENCIL_SIZE : 0x8217,
        FRAMEBUFFER_DEFAULT : 0x8218,
        UNSIGNED_INT_24_8 : 0x84FA,
        DEPTH24_STENCIL8 : 0x88F0,
        UNSIGNED_NORMALIZED : 0x8C17,
        DRAW_FRAMEBUFFER_BINDING : 0x8CA6, // Same as FRAMEBUFFER_BINDING
        READ_FRAMEBUFFER : 0x8CA8,
        DRAW_FRAMEBUFFER : 0x8CA9,
        READ_FRAMEBUFFER_BINDING : 0x8CAA,
        RENDERBUFFER_SAMPLES : 0x8CAB,
        FRAMEBUFFER_ATTACHMENT_TEXTURE_LAYER : 0x8CD4,
        MAX_COLOR_ATTACHMENTS : 0x8CDF,
        COLOR_ATTACHMENT1 : 0x8CE1,
        COLOR_ATTACHMENT2 : 0x8CE2,
        COLOR_ATTACHMENT3 : 0x8CE3,
        COLOR_ATTACHMENT4 : 0x8CE4,
        COLOR_ATTACHMENT5 : 0x8CE5,
        COLOR_ATTACHMENT6 : 0x8CE6,
        COLOR_ATTACHMENT7 : 0x8CE7,
        COLOR_ATTACHMENT8 : 0x8CE8,
        COLOR_ATTACHMENT9 : 0x8CE9,
        COLOR_ATTACHMENT10 : 0x8CEA,
        COLOR_ATTACHMENT11 : 0x8CEB,
        COLOR_ATTACHMENT12 : 0x8CEC,
        COLOR_ATTACHMENT13 : 0x8CED,
        COLOR_ATTACHMENT14 : 0x8CEE,
        COLOR_ATTACHMENT15 : 0x8CEF,
        FRAMEBUFFER_INCOMPLETE_MULTISAMPLE : 0x8D56,
        MAX_SAMPLES : 0x8D57,
        HALF_FLOAT : 0x140B,
        RG : 0x8227,
        RG_INTEGER : 0x8228,
        R8 : 0x8229,
        RG8 : 0x822B,
        R16F : 0x822D,
        R32F : 0x822E,
        RG16F : 0x822F,
        RG32F : 0x8230,
        R8I : 0x8231,
        R8UI : 0x8232,
        R16I : 0x8233,
        R16UI : 0x8234,
        R32I : 0x8235,
        R32UI : 0x8236,
        RG8I : 0x8237,
        RG8UI : 0x8238,
        RG16I : 0x8239,
        RG16UI : 0x823A,
        RG32I : 0x823B,
        RG32UI : 0x823C,
        VERTEX_ARRAY_BINDING : 0x85B5,
        R8_SNORM : 0x8F94,
        RG8_SNORM : 0x8F95,
        RGB8_SNORM : 0x8F96,
        RGBA8_SNORM : 0x8F97,
        SIGNED_NORMALIZED : 0x8F9C,
        COPY_READ_BUFFER : 0x8F36,
        COPY_WRITE_BUFFER : 0x8F37,
        COPY_READ_BUFFER_BINDING : 0x8F36, // Same as COPY_READ_BUFFER
        COPY_WRITE_BUFFER_BINDING : 0x8F37, // Same as COPY_WRITE_BUFFER
        UNIFORM_BUFFER : 0x8A11,
        UNIFORM_BUFFER_BINDING : 0x8A28,
        UNIFORM_BUFFER_START : 0x8A29,
        UNIFORM_BUFFER_SIZE : 0x8A2A,
        MAX_VERTEX_UNIFORM_BLOCKS : 0x8A2B,
        MAX_FRAGMENT_UNIFORM_BLOCKS : 0x8A2D,
        MAX_COMBINED_UNIFORM_BLOCKS : 0x8A2E,
        MAX_UNIFORM_BUFFER_BINDINGS : 0x8A2F,
        MAX_UNIFORM_BLOCK_SIZE : 0x8A30,
        MAX_COMBINED_VERTEX_UNIFORM_COMPONENTS : 0x8A31,
        MAX_COMBINED_FRAGMENT_UNIFORM_COMPONENTS : 0x8A33,
        UNIFORM_BUFFER_OFFSET_ALIGNMENT : 0x8A34,
        ACTIVE_UNIFORM_BLOCKS : 0x8A36,
        UNIFORM_TYPE : 0x8A37,
        UNIFORM_SIZE : 0x8A38,
        UNIFORM_BLOCK_INDEX : 0x8A3A,
        UNIFORM_OFFSET : 0x8A3B,
        UNIFORM_ARRAY_STRIDE : 0x8A3C,
        UNIFORM_MATRIX_STRIDE : 0x8A3D,
        UNIFORM_IS_ROW_MAJOR : 0x8A3E,
        UNIFORM_BLOCK_BINDING : 0x8A3F,
        UNIFORM_BLOCK_DATA_SIZE : 0x8A40,
        UNIFORM_BLOCK_ACTIVE_UNIFORMS : 0x8A42,
        UNIFORM_BLOCK_ACTIVE_UNIFORM_INDICES : 0x8A43,
        UNIFORM_BLOCK_REFERENCED_BY_VERTEX_SHADER : 0x8A44,
        UNIFORM_BLOCK_REFERENCED_BY_FRAGMENT_SHADER : 0x8A46,
        INVALID_INDEX : 0xFFFFFFFF,
        MAX_VERTEX_OUTPUT_COMPONENTS : 0x9122,
        MAX_FRAGMENT_INPUT_COMPONENTS : 0x9125,
        MAX_SERVER_WAIT_TIMEOUT : 0x9111,
        OBJECT_TYPE : 0x9112,
        SYNC_CONDITION : 0x9113,
        SYNC_STATUS : 0x9114,
        SYNC_FLAGS : 0x9115,
        SYNC_FENCE : 0x9116,
        SYNC_GPU_COMMANDS_COMPLETE : 0x9117,
        UNSIGNALED : 0x9118,
        SIGNALED : 0x9119,
        ALREADY_SIGNALED : 0x911A,
        TIMEOUT_EXPIRED : 0x911B,
        CONDITION_SATISFIED : 0x911C,
        WAIT_FAILED : 0x911D,
        SYNC_FLUSH_COMMANDS_BIT : 0x00000001,
        VERTEX_ATTRIB_ARRAY_DIVISOR : 0x88FE,
        ANY_SAMPLES_PASSED : 0x8C2F,
        ANY_SAMPLES_PASSED_CONSERVATIVE : 0x8D6A,
        SAMPLER_BINDING : 0x8919,
        RGB10_A2UI : 0x906F,
        INT_2_10_10_10_REV : 0x8D9F,
        TRANSFORM_FEEDBACK : 0x8E22,
        TRANSFORM_FEEDBACK_PAUSED : 0x8E23,
        TRANSFORM_FEEDBACK_ACTIVE : 0x8E24,
        TRANSFORM_FEEDBACK_BINDING : 0x8E25,
        COMPRESSED_R11_EAC : 0x9270,
        COMPRESSED_SIGNED_R11_EAC : 0x9271,
        COMPRESSED_RG11_EAC : 0x9272,
        COMPRESSED_SIGNED_RG11_EAC : 0x9273,
        COMPRESSED_RGB8_ETC2 : 0x9274,
        COMPRESSED_SRGB8_ETC2 : 0x9275,
        COMPRESSED_RGB8_PUNCHTHROUGH_ALPHA1_ETC2 : 0x9276,
        COMPRESSED_SRGB8_PUNCHTHROUGH_ALPHA1_ETC2 : 0x9277,
        COMPRESSED_RGBA8_ETC2_EAC : 0x9278,
        COMPRESSED_SRGB8_ALPHA8_ETC2_EAC : 0x9279,
        TEXTURE_IMMUTABLE_FORMAT : 0x912F,
        MAX_ELEMENT_INDEX : 0x8D6B,
        TEXTURE_IMMUTABLE_LEVELS : 0x82DF,

        // Extensions
        MAX_TEXTURE_MAX_ANISOTROPY_EXT : 0x84FF
    };

    return freezeObject(WebGLConstants);
});

define('Core/ComponentDatatype',[
        './defaultValue',
        './defined',
        './DeveloperError',
        './FeatureDetection',
        './freezeObject',
        './WebGLConstants'
    ], function(
        defaultValue,
        defined,
        DeveloperError,
        FeatureDetection,
        freezeObject,
        WebGLConstants) {
    'use strict';

    // Bail out if the browser doesn't support typed arrays, to prevent the setup function
    // from failing, since we won't be able to create a WebGL context anyway.
    if (!FeatureDetection.supportsTypedArrays()) {
        return {};
    }

    /**
     * WebGL component datatypes.  Components are intrinsics,
     * which form attributes, which form vertices.
     *
     * @exports ComponentDatatype
     */
    var ComponentDatatype = {
        /**
         * 8-bit signed byte corresponding to <code>gl.BYTE</code> and the type
         * of an element in <code>Int8Array</code>.
         *
         * @type {Number}
         * @constant
         */
        BYTE : WebGLConstants.BYTE,

        /**
         * 8-bit unsigned byte corresponding to <code>UNSIGNED_BYTE</code> and the type
         * of an element in <code>Uint8Array</code>.
         *
         * @type {Number}
         * @constant
         */
        UNSIGNED_BYTE : WebGLConstants.UNSIGNED_BYTE,

        /**
         * 16-bit signed short corresponding to <code>SHORT</code> and the type
         * of an element in <code>Int16Array</code>.
         *
         * @type {Number}
         * @constant
         */
        SHORT : WebGLConstants.SHORT,

        /**
         * 16-bit unsigned short corresponding to <code>UNSIGNED_SHORT</code> and the type
         * of an element in <code>Uint16Array</code>.
         *
         * @type {Number}
         * @constant
         */
        UNSIGNED_SHORT : WebGLConstants.UNSIGNED_SHORT,

        /**
         * 32-bit signed int corresponding to <code>INT</code> and the type
         * of an element in <code>Int32Array</code>.
         *
         * @memberOf ComponentDatatype
         *
         * @type {Number}
         * @constant
         */
        INT : WebGLConstants.INT,

        /**
         * 32-bit unsigned int corresponding to <code>UNSIGNED_INT</code> and the type
         * of an element in <code>Uint32Array</code>.
         *
         * @memberOf ComponentDatatype
         *
         * @type {Number}
         * @constant
         */
        UNSIGNED_INT : WebGLConstants.UNSIGNED_INT,

        /**
         * 32-bit floating-point corresponding to <code>FLOAT</code> and the type
         * of an element in <code>Float32Array</code>.
         *
         * @type {Number}
         * @constant
         */
        FLOAT : WebGLConstants.FLOAT,

        /**
         * 64-bit floating-point corresponding to <code>gl.DOUBLE</code> (in Desktop OpenGL;
         * this is not supported in WebGL, and is emulated in Cesium via {@link GeometryPipeline.encodeAttribute})
         * and the type of an element in <code>Float64Array</code>.
         *
         * @memberOf ComponentDatatype
         *
         * @type {Number}
         * @constant
         * @default 0x140A
         */
        DOUBLE : WebGLConstants.DOUBLE
    };

    /**
     * Returns the size, in bytes, of the corresponding datatype.
     *
     * @param {ComponentDatatype} componentDatatype The component datatype to get the size of.
     * @returns {Number} The size in bytes.
     *
     * @exception {DeveloperError} componentDatatype is not a valid value.
     *
     * @example
     * // Returns Int8Array.BYTES_PER_ELEMENT
     * var size = Cesium.ComponentDatatype.getSizeInBytes(Cesium.ComponentDatatype.BYTE);
     */
    ComponentDatatype.getSizeInBytes = function(componentDatatype){
                if (!defined(componentDatatype)) {
            throw new DeveloperError('value is required.');
        }
        
        switch (componentDatatype) {
        case ComponentDatatype.BYTE:
            return Int8Array.BYTES_PER_ELEMENT;
        case ComponentDatatype.UNSIGNED_BYTE:
            return Uint8Array.BYTES_PER_ELEMENT;
        case ComponentDatatype.SHORT:
            return Int16Array.BYTES_PER_ELEMENT;
        case ComponentDatatype.UNSIGNED_SHORT:
            return Uint16Array.BYTES_PER_ELEMENT;
        case ComponentDatatype.INT:
            return Int32Array.BYTES_PER_ELEMENT;
        case ComponentDatatype.UNSIGNED_INT:
            return Uint32Array.BYTES_PER_ELEMENT;
        case ComponentDatatype.FLOAT:
            return Float32Array.BYTES_PER_ELEMENT;
        case ComponentDatatype.DOUBLE:
            return Float64Array.BYTES_PER_ELEMENT;
                default:
            throw new DeveloperError('componentDatatype is not a valid value.');
                }
    };

    /**
     * Gets the {@link ComponentDatatype} for the provided TypedArray instance.
     *
     * @param {TypedArray} array The typed array.
     * @returns {ComponentDatatype} The ComponentDatatype for the provided array, or undefined if the array is not a TypedArray.
     */
    ComponentDatatype.fromTypedArray = function(array) {
        if (array instanceof Int8Array) {
            return ComponentDatatype.BYTE;
        }
        if (array instanceof Uint8Array) {
            return ComponentDatatype.UNSIGNED_BYTE;
        }
        if (array instanceof Int16Array) {
            return ComponentDatatype.SHORT;
        }
        if (array instanceof Uint16Array) {
            return ComponentDatatype.UNSIGNED_SHORT;
        }
        if (array instanceof Int32Array) {
            return ComponentDatatype.INT;
        }
        if (array instanceof Uint32Array) {
            return ComponentDatatype.UNSIGNED_INT;
        }
        if (array instanceof Float32Array) {
            return ComponentDatatype.FLOAT;
        }
        if (array instanceof Float64Array) {
            return ComponentDatatype.DOUBLE;
        }
    };

    /**
     * Validates that the provided component datatype is a valid {@link ComponentDatatype}
     *
     * @param {ComponentDatatype} componentDatatype The component datatype to validate.
     * @returns {Boolean} <code>true</code> if the provided component datatype is a valid value; otherwise, <code>false</code>.
     *
     * @example
     * if (!Cesium.ComponentDatatype.validate(componentDatatype)) {
     *   throw new Cesium.DeveloperError('componentDatatype must be a valid value.');
     * }
     */
    ComponentDatatype.validate = function(componentDatatype) {
        return defined(componentDatatype) &&
               (componentDatatype === ComponentDatatype.BYTE ||
                componentDatatype === ComponentDatatype.UNSIGNED_BYTE ||
                componentDatatype === ComponentDatatype.SHORT ||
                componentDatatype === ComponentDatatype.UNSIGNED_SHORT ||
                componentDatatype === ComponentDatatype.INT ||
                componentDatatype === ComponentDatatype.UNSIGNED_INT ||
                componentDatatype === ComponentDatatype.FLOAT ||
                componentDatatype === ComponentDatatype.DOUBLE);
    };

    /**
     * Creates a typed array corresponding to component data type.
     *
     * @param {ComponentDatatype} componentDatatype The component data type.
     * @param {Number|Array} valuesOrLength The length of the array to create or an array.
     * @returns {Int8Array|Uint8Array|Int16Array|Uint16Array|Int32Array|Uint32Array|Float32Array|Float64Array} A typed array.
     *
     * @exception {DeveloperError} componentDatatype is not a valid value.
     *
     * @example
     * // creates a Float32Array with length of 100
     * var typedArray = Cesium.ComponentDatatype.createTypedArray(Cesium.ComponentDatatype.FLOAT, 100);
     */
    ComponentDatatype.createTypedArray = function(componentDatatype, valuesOrLength) {
                if (!defined(componentDatatype)) {
            throw new DeveloperError('componentDatatype is required.');
        }
        if (!defined(valuesOrLength)) {
            throw new DeveloperError('valuesOrLength is required.');
        }
        
        switch (componentDatatype) {
        case ComponentDatatype.BYTE:
            return new Int8Array(valuesOrLength);
        case ComponentDatatype.UNSIGNED_BYTE:
            return new Uint8Array(valuesOrLength);
        case ComponentDatatype.SHORT:
            return new Int16Array(valuesOrLength);
        case ComponentDatatype.UNSIGNED_SHORT:
            return new Uint16Array(valuesOrLength);
        case ComponentDatatype.INT:
            return new Int32Array(valuesOrLength);
        case ComponentDatatype.UNSIGNED_INT:
            return new Uint32Array(valuesOrLength);
        case ComponentDatatype.FLOAT:
            return new Float32Array(valuesOrLength);
        case ComponentDatatype.DOUBLE:
            return new Float64Array(valuesOrLength);
                default:
            throw new DeveloperError('componentDatatype is not a valid value.');
                }
    };

    /**
     * Creates a typed view of an array of bytes.
     *
     * @param {ComponentDatatype} componentDatatype The type of the view to create.
     * @param {ArrayBuffer} buffer The buffer storage to use for the view.
     * @param {Number} [byteOffset] The offset, in bytes, to the first element in the view.
     * @param {Number} [length] The number of elements in the view.
     * @returns {Int8Array|Uint8Array|Int16Array|Uint16Array|Int32Array|Uint32Array|Float32Array|Float64Array} A typed array view of the buffer.
     *
     * @exception {DeveloperError} componentDatatype is not a valid value.
     */
    ComponentDatatype.createArrayBufferView = function(componentDatatype, buffer, byteOffset, length) {
                if (!defined(componentDatatype)) {
            throw new DeveloperError('componentDatatype is required.');
        }
        if (!defined(buffer)) {
            throw new DeveloperError('buffer is required.');
        }
        
        byteOffset = defaultValue(byteOffset, 0);
        length = defaultValue(length, (buffer.byteLength - byteOffset) / ComponentDatatype.getSizeInBytes(componentDatatype));

        switch (componentDatatype) {
        case ComponentDatatype.BYTE:
            return new Int8Array(buffer, byteOffset, length);
        case ComponentDatatype.UNSIGNED_BYTE:
            return new Uint8Array(buffer, byteOffset, length);
        case ComponentDatatype.SHORT:
            return new Int16Array(buffer, byteOffset, length);
        case ComponentDatatype.UNSIGNED_SHORT:
            return new Uint16Array(buffer, byteOffset, length);
        case ComponentDatatype.INT:
            return new Int32Array(buffer, byteOffset, length);
        case ComponentDatatype.UNSIGNED_INT:
            return new Uint32Array(buffer, byteOffset, length);
        case ComponentDatatype.FLOAT:
            return new Float32Array(buffer, byteOffset, length);
        case ComponentDatatype.DOUBLE:
            return new Float64Array(buffer, byteOffset, length);
                default:
            throw new DeveloperError('componentDatatype is not a valid value.');
                }
    };

    /**
     * Get the ComponentDatatype from its name.
     *
     * @param {String} name The name of the ComponentDatatype.
     * @returns {ComponentDatatype} The ComponentDatatype.
     *
     * @exception {DeveloperError} name is not a valid value.
     */
    ComponentDatatype.fromName = function(name) {
        switch (name) {
            case 'BYTE':
                return ComponentDatatype.BYTE;
            case 'UNSIGNED_BYTE':
                return ComponentDatatype.UNSIGNED_BYTE;
            case 'SHORT':
                return ComponentDatatype.SHORT;
            case 'UNSIGNED_SHORT':
                return ComponentDatatype.UNSIGNED_SHORT;
            case 'INT':
                return ComponentDatatype.INT;
            case 'UNSIGNED_INT':
                return ComponentDatatype.UNSIGNED_INT;
            case 'FLOAT':
                return ComponentDatatype.FLOAT;
            case 'DOUBLE':
                return ComponentDatatype.DOUBLE;
                        default:
                throw new DeveloperError('name is not a valid value.');
                    }
    };

    return freezeObject(ComponentDatatype);
});

define('Core/Quaternion',[
        './Cartesian3',
        './Check',
        './defaultValue',
        './defined',
        './FeatureDetection',
        './freezeObject',
        './Math',
        './Matrix3'
    ], function(
        Cartesian3,
        Check,
        defaultValue,
        defined,
        FeatureDetection,
        freezeObject,
        CesiumMath,
        Matrix3) {
    'use strict';

    /**
     * A set of 4-dimensional coordinates used to represent rotation in 3-dimensional space.
     * @alias Quaternion
     * @constructor
     *
     * @param {Number} [x=0.0] The X component.
     * @param {Number} [y=0.0] The Y component.
     * @param {Number} [z=0.0] The Z component.
     * @param {Number} [w=0.0] The W component.
     *
     * @see PackableForInterpolation
     */
    function Quaternion(x, y, z, w) {
        /**
         * The X component.
         * @type {Number}
         * @default 0.0
         */
        this.x = defaultValue(x, 0.0);

        /**
         * The Y component.
         * @type {Number}
         * @default 0.0
         */
        this.y = defaultValue(y, 0.0);

        /**
         * The Z component.
         * @type {Number}
         * @default 0.0
         */
        this.z = defaultValue(z, 0.0);

        /**
         * The W component.
         * @type {Number}
         * @default 0.0
         */
        this.w = defaultValue(w, 0.0);
    }

    var fromAxisAngleScratch = new Cartesian3();

    /**
     * Computes a quaternion representing a rotation around an axis.
     *
     * @param {Cartesian3} axis The axis of rotation.
     * @param {Number} angle The angle in radians to rotate around the axis.
     * @param {Quaternion} [result] The object onto which to store the result.
     * @returns {Quaternion} The modified result parameter or a new Quaternion instance if one was not provided.
     */
    Quaternion.fromAxisAngle = function(axis, angle, result) {
                Check.typeOf.object('axis', axis);
        Check.typeOf.number('angle', angle);
        
        var halfAngle = angle / 2.0;
        var s = Math.sin(halfAngle);
        fromAxisAngleScratch = Cartesian3.normalize(axis, fromAxisAngleScratch);

        var x = fromAxisAngleScratch.x * s;
        var y = fromAxisAngleScratch.y * s;
        var z = fromAxisAngleScratch.z * s;
        var w = Math.cos(halfAngle);
        if (!defined(result)) {
            return new Quaternion(x, y, z, w);
        }
        result.x = x;
        result.y = y;
        result.z = z;
        result.w = w;
        return result;
    };

    var fromRotationMatrixNext = [1, 2, 0];
    var fromRotationMatrixQuat = new Array(3);
    /**
     * Computes a Quaternion from the provided Matrix3 instance.
     *
     * @param {Matrix3} matrix The rotation matrix.
     * @param {Quaternion} [result] The object onto which to store the result.
     * @returns {Quaternion} The modified result parameter or a new Quaternion instance if one was not provided.
     *
     * @see Matrix3.fromQuaternion
     */
    Quaternion.fromRotationMatrix = function(matrix, result) {
                Check.typeOf.object('matrix', matrix);
        
        var root;
        var x;
        var y;
        var z;
        var w;

        var m00 = matrix[Matrix3.COLUMN0ROW0];
        var m11 = matrix[Matrix3.COLUMN1ROW1];
        var m22 = matrix[Matrix3.COLUMN2ROW2];
        var trace = m00 + m11 + m22;

        if (trace > 0.0) {
            // |w| > 1/2, may as well choose w > 1/2
            root = Math.sqrt(trace + 1.0); // 2w
            w = 0.5 * root;
            root = 0.5 / root; // 1/(4w)

            x = (matrix[Matrix3.COLUMN1ROW2] - matrix[Matrix3.COLUMN2ROW1]) * root;
            y = (matrix[Matrix3.COLUMN2ROW0] - matrix[Matrix3.COLUMN0ROW2]) * root;
            z = (matrix[Matrix3.COLUMN0ROW1] - matrix[Matrix3.COLUMN1ROW0]) * root;
        } else {
            // |w| <= 1/2
            var next = fromRotationMatrixNext;

            var i = 0;
            if (m11 > m00) {
                i = 1;
            }
            if (m22 > m00 && m22 > m11) {
                i = 2;
            }
            var j = next[i];
            var k = next[j];

            root = Math.sqrt(matrix[Matrix3.getElementIndex(i, i)] - matrix[Matrix3.getElementIndex(j, j)] - matrix[Matrix3.getElementIndex(k, k)] + 1.0);

            var quat = fromRotationMatrixQuat;
            quat[i] = 0.5 * root;
            root = 0.5 / root;
            w = (matrix[Matrix3.getElementIndex(k, j)] - matrix[Matrix3.getElementIndex(j, k)]) * root;
            quat[j] = (matrix[Matrix3.getElementIndex(j, i)] + matrix[Matrix3.getElementIndex(i, j)]) * root;
            quat[k] = (matrix[Matrix3.getElementIndex(k, i)] + matrix[Matrix3.getElementIndex(i, k)]) * root;

            x = -quat[0];
            y = -quat[1];
            z = -quat[2];
        }

        if (!defined(result)) {
            return new Quaternion(x, y, z, w);
        }
        result.x = x;
        result.y = y;
        result.z = z;
        result.w = w;
        return result;
    };

    var scratchHPRQuaternion = new Quaternion();
    var scratchHeadingQuaternion = new Quaternion();
    var scratchPitchQuaternion = new Quaternion();
    var scratchRollQuaternion = new Quaternion();

    /**
     * Computes a rotation from the given heading, pitch and roll angles. Heading is the rotation about the
     * negative z axis. Pitch is the rotation about the negative y axis. Roll is the rotation about
     * the positive x axis.
     *
     * @param {HeadingPitchRoll} headingPitchRoll The rotation expressed as a heading, pitch and roll.
     * @param {Quaternion} [result] The object onto which to store the result.
     * @returns {Quaternion} The modified result parameter or a new Quaternion instance if none was provided.
     */
    Quaternion.fromHeadingPitchRoll = function(headingPitchRoll, result) {
                Check.typeOf.object('headingPitchRoll', headingPitchRoll);
        
        scratchRollQuaternion = Quaternion.fromAxisAngle(Cartesian3.UNIT_X, headingPitchRoll.roll, scratchHPRQuaternion);
        scratchPitchQuaternion = Quaternion.fromAxisAngle(Cartesian3.UNIT_Y, -headingPitchRoll.pitch, result);
        result = Quaternion.multiply(scratchPitchQuaternion, scratchRollQuaternion, scratchPitchQuaternion);
        scratchHeadingQuaternion = Quaternion.fromAxisAngle(Cartesian3.UNIT_Z, -headingPitchRoll.heading, scratchHPRQuaternion);
        return Quaternion.multiply(scratchHeadingQuaternion, result, result);
    };

    var sampledQuaternionAxis = new Cartesian3();
    var sampledQuaternionRotation = new Cartesian3();
    var sampledQuaternionTempQuaternion = new Quaternion();
    var sampledQuaternionQuaternion0 = new Quaternion();
    var sampledQuaternionQuaternion0Conjugate = new Quaternion();

    /**
     * The number of elements used to pack the object into an array.
     * @type {Number}
     */
    Quaternion.packedLength = 4;

    /**
     * Stores the provided instance into the provided array.
     *
     * @param {Quaternion} value The value to pack.
     * @param {Number[]} array The array to pack into.
     * @param {Number} [startingIndex=0] The index into the array at which to start packing the elements.
     *
     * @returns {Number[]} The array that was packed into
     */
    Quaternion.pack = function(value, array, startingIndex) {
                Check.typeOf.object('value', value);
        Check.defined('array', array);
        
        startingIndex = defaultValue(startingIndex, 0);

        array[startingIndex++] = value.x;
        array[startingIndex++] = value.y;
        array[startingIndex++] = value.z;
        array[startingIndex] = value.w;

        return array;
    };

    /**
     * Retrieves an instance from a packed array.
     *
     * @param {Number[]} array The packed array.
     * @param {Number} [startingIndex=0] The starting index of the element to be unpacked.
     * @param {Quaternion} [result] The object into which to store the result.
     * @returns {Quaternion} The modified result parameter or a new Quaternion instance if one was not provided.
     */
    Quaternion.unpack = function(array, startingIndex, result) {
                Check.defined('array', array);
        
        startingIndex = defaultValue(startingIndex, 0);

        if (!defined(result)) {
            result = new Quaternion();
        }
        result.x = array[startingIndex];
        result.y = array[startingIndex + 1];
        result.z = array[startingIndex + 2];
        result.w = array[startingIndex + 3];
        return result;
    };

    /**
     * The number of elements used to store the object into an array in its interpolatable form.
     * @type {Number}
     */
    Quaternion.packedInterpolationLength = 3;

    /**
     * Converts a packed array into a form suitable for interpolation.
     *
     * @param {Number[]} packedArray The packed array.
     * @param {Number} [startingIndex=0] The index of the first element to be converted.
     * @param {Number} [lastIndex=packedArray.length] The index of the last element to be converted.
     * @param {Number[]} result The object into which to store the result.
     */
    Quaternion.convertPackedArrayForInterpolation = function(packedArray, startingIndex, lastIndex, result) {
        Quaternion.unpack(packedArray, lastIndex * 4, sampledQuaternionQuaternion0Conjugate);
        Quaternion.conjugate(sampledQuaternionQuaternion0Conjugate, sampledQuaternionQuaternion0Conjugate);

        for (var i = 0, len = lastIndex - startingIndex + 1; i < len; i++) {
            var offset = i * 3;
            Quaternion.unpack(packedArray, (startingIndex + i) * 4, sampledQuaternionTempQuaternion);

            Quaternion.multiply(sampledQuaternionTempQuaternion, sampledQuaternionQuaternion0Conjugate, sampledQuaternionTempQuaternion);

            if (sampledQuaternionTempQuaternion.w < 0) {
                Quaternion.negate(sampledQuaternionTempQuaternion, sampledQuaternionTempQuaternion);
            }

            Quaternion.computeAxis(sampledQuaternionTempQuaternion, sampledQuaternionAxis);
            var angle = Quaternion.computeAngle(sampledQuaternionTempQuaternion);
            result[offset] = sampledQuaternionAxis.x * angle;
            result[offset + 1] = sampledQuaternionAxis.y * angle;
            result[offset + 2] = sampledQuaternionAxis.z * angle;
        }
    };

    /**
     * Retrieves an instance from a packed array converted with {@link convertPackedArrayForInterpolation}.
     *
     * @param {Number[]} array The array previously packed for interpolation.
     * @param {Number[]} sourceArray The original packed array.
     * @param {Number} [firstIndex=0] The firstIndex used to convert the array.
     * @param {Number} [lastIndex=packedArray.length] The lastIndex used to convert the array.
     * @param {Quaternion} [result] The object into which to store the result.
     * @returns {Quaternion} The modified result parameter or a new Quaternion instance if one was not provided.
     */
    Quaternion.unpackInterpolationResult = function(array, sourceArray, firstIndex, lastIndex, result) {
        if (!defined(result)) {
            result = new Quaternion();
        }
        Cartesian3.fromArray(array, 0, sampledQuaternionRotation);
        var magnitude = Cartesian3.magnitude(sampledQuaternionRotation);

        Quaternion.unpack(sourceArray, lastIndex * 4, sampledQuaternionQuaternion0);

        if (magnitude === 0) {
            Quaternion.clone(Quaternion.IDENTITY, sampledQuaternionTempQuaternion);
        } else {
            Quaternion.fromAxisAngle(sampledQuaternionRotation, magnitude, sampledQuaternionTempQuaternion);
        }

        return Quaternion.multiply(sampledQuaternionTempQuaternion, sampledQuaternionQuaternion0, result);
    };

    /**
     * Duplicates a Quaternion instance.
     *
     * @param {Quaternion} quaternion The quaternion to duplicate.
     * @param {Quaternion} [result] The object onto which to store the result.
     * @returns {Quaternion} The modified result parameter or a new Quaternion instance if one was not provided. (Returns undefined if quaternion is undefined)
     */
    Quaternion.clone = function(quaternion, result) {
        if (!defined(quaternion)) {
            return undefined;
        }

        if (!defined(result)) {
            return new Quaternion(quaternion.x, quaternion.y, quaternion.z, quaternion.w);
        }

        result.x = quaternion.x;
        result.y = quaternion.y;
        result.z = quaternion.z;
        result.w = quaternion.w;
        return result;
    };

    /**
     * Computes the conjugate of the provided quaternion.
     *
     * @param {Quaternion} quaternion The quaternion to conjugate.
     * @param {Quaternion} result The object onto which to store the result.
     * @returns {Quaternion} The modified result parameter.
     */
    Quaternion.conjugate = function(quaternion, result) {
                Check.typeOf.object('quaternion', quaternion);
        Check.typeOf.object('result', result);
        
        result.x = -quaternion.x;
        result.y = -quaternion.y;
        result.z = -quaternion.z;
        result.w = quaternion.w;
        return result;
    };

    /**
     * Computes magnitude squared for the provided quaternion.
     *
     * @param {Quaternion} quaternion The quaternion to conjugate.
     * @returns {Number} The magnitude squared.
     */
    Quaternion.magnitudeSquared = function(quaternion) {
                Check.typeOf.object('quaternion', quaternion);
        
        return quaternion.x * quaternion.x + quaternion.y * quaternion.y + quaternion.z * quaternion.z + quaternion.w * quaternion.w;
    };

    /**
     * Computes magnitude for the provided quaternion.
     *
     * @param {Quaternion} quaternion The quaternion to conjugate.
     * @returns {Number} The magnitude.
     */
    Quaternion.magnitude = function(quaternion) {
        return Math.sqrt(Quaternion.magnitudeSquared(quaternion));
    };

    /**
     * Computes the normalized form of the provided quaternion.
     *
     * @param {Quaternion} quaternion The quaternion to normalize.
     * @param {Quaternion} result The object onto which to store the result.
     * @returns {Quaternion} The modified result parameter.
     */
    Quaternion.normalize = function(quaternion, result) {
                Check.typeOf.object('result', result);
        
        var inverseMagnitude = 1.0 / Quaternion.magnitude(quaternion);
        var x = quaternion.x * inverseMagnitude;
        var y = quaternion.y * inverseMagnitude;
        var z = quaternion.z * inverseMagnitude;
        var w = quaternion.w * inverseMagnitude;

        result.x = x;
        result.y = y;
        result.z = z;
        result.w = w;
        return result;
    };

    /**
     * Computes the inverse of the provided quaternion.
     *
     * @param {Quaternion} quaternion The quaternion to normalize.
     * @param {Quaternion} result The object onto which to store the result.
     * @returns {Quaternion} The modified result parameter.
     */
    Quaternion.inverse = function(quaternion, result) {
                Check.typeOf.object('result', result);
        
        var magnitudeSquared = Quaternion.magnitudeSquared(quaternion);
        result = Quaternion.conjugate(quaternion, result);
        return Quaternion.multiplyByScalar(result, 1.0 / magnitudeSquared, result);
    };

    /**
     * Computes the componentwise sum of two quaternions.
     *
     * @param {Quaternion} left The first quaternion.
     * @param {Quaternion} right The second quaternion.
     * @param {Quaternion} result The object onto which to store the result.
     * @returns {Quaternion} The modified result parameter.
     */
    Quaternion.add = function(left, right, result) {
                Check.typeOf.object('left', left);
        Check.typeOf.object('right', right);
        Check.typeOf.object('result', result);
        
        result.x = left.x + right.x;
        result.y = left.y + right.y;
        result.z = left.z + right.z;
        result.w = left.w + right.w;
        return result;
    };

    /**
     * Computes the componentwise difference of two quaternions.
     *
     * @param {Quaternion} left The first quaternion.
     * @param {Quaternion} right The second quaternion.
     * @param {Quaternion} result The object onto which to store the result.
     * @returns {Quaternion} The modified result parameter.
     */
    Quaternion.subtract = function(left, right, result) {
                Check.typeOf.object('left', left);
        Check.typeOf.object('right', right);
        Check.typeOf.object('result', result);
        
        result.x = left.x - right.x;
        result.y = left.y - right.y;
        result.z = left.z - right.z;
        result.w = left.w - right.w;
        return result;
    };

    /**
     * Negates the provided quaternion.
     *
     * @param {Quaternion} quaternion The quaternion to be negated.
     * @param {Quaternion} result The object onto which to store the result.
     * @returns {Quaternion} The modified result parameter.
     */
    Quaternion.negate = function(quaternion, result) {
                Check.typeOf.object('quaternion', quaternion);
        Check.typeOf.object('result', result);
        
        result.x = -quaternion.x;
        result.y = -quaternion.y;
        result.z = -quaternion.z;
        result.w = -quaternion.w;
        return result;
    };

    /**
     * Computes the dot (scalar) product of two quaternions.
     *
     * @param {Quaternion} left The first quaternion.
     * @param {Quaternion} right The second quaternion.
     * @returns {Number} The dot product.
     */
    Quaternion.dot = function(left, right) {
                Check.typeOf.object('left', left);
        Check.typeOf.object('right', right);
        
        return left.x * right.x + left.y * right.y + left.z * right.z + left.w * right.w;
    };

    /**
     * Computes the product of two quaternions.
     *
     * @param {Quaternion} left The first quaternion.
     * @param {Quaternion} right The second quaternion.
     * @param {Quaternion} result The object onto which to store the result.
     * @returns {Quaternion} The modified result parameter.
     */
    Quaternion.multiply = function(left, right, result) {
                Check.typeOf.object('left', left);
        Check.typeOf.object('right', right);
        Check.typeOf.object('result', result);
        
        var leftX = left.x;
        var leftY = left.y;
        var leftZ = left.z;
        var leftW = left.w;

        var rightX = right.x;
        var rightY = right.y;
        var rightZ = right.z;
        var rightW = right.w;

        var x = leftW * rightX + leftX * rightW + leftY * rightZ - leftZ * rightY;
        var y = leftW * rightY - leftX * rightZ + leftY * rightW + leftZ * rightX;
        var z = leftW * rightZ + leftX * rightY - leftY * rightX + leftZ * rightW;
        var w = leftW * rightW - leftX * rightX - leftY * rightY - leftZ * rightZ;

        result.x = x;
        result.y = y;
        result.z = z;
        result.w = w;
        return result;
    };

    /**
     * Multiplies the provided quaternion componentwise by the provided scalar.
     *
     * @param {Quaternion} quaternion The quaternion to be scaled.
     * @param {Number} scalar The scalar to multiply with.
     * @param {Quaternion} result The object onto which to store the result.
     * @returns {Quaternion} The modified result parameter.
     */
    Quaternion.multiplyByScalar = function(quaternion, scalar, result) {
                Check.typeOf.object('quaternion', quaternion);
        Check.typeOf.number('scalar', scalar);
        Check.typeOf.object('result', result);
        
        result.x = quaternion.x * scalar;
        result.y = quaternion.y * scalar;
        result.z = quaternion.z * scalar;
        result.w = quaternion.w * scalar;
        return result;
    };

    /**
     * Divides the provided quaternion componentwise by the provided scalar.
     *
     * @param {Quaternion} quaternion The quaternion to be divided.
     * @param {Number} scalar The scalar to divide by.
     * @param {Quaternion} result The object onto which to store the result.
     * @returns {Quaternion} The modified result parameter.
     */
    Quaternion.divideByScalar = function(quaternion, scalar, result) {
                Check.typeOf.object('quaternion', quaternion);
        Check.typeOf.number('scalar', scalar);
        Check.typeOf.object('result', result);
        
        result.x = quaternion.x / scalar;
        result.y = quaternion.y / scalar;
        result.z = quaternion.z / scalar;
        result.w = quaternion.w / scalar;
        return result;
    };

    /**
     * Computes the axis of rotation of the provided quaternion.
     *
     * @param {Quaternion} quaternion The quaternion to use.
     * @param {Cartesian3} result The object onto which to store the result.
     * @returns {Cartesian3} The modified result parameter.
     */
    Quaternion.computeAxis = function(quaternion, result) {
                Check.typeOf.object('quaternion', quaternion);
        Check.typeOf.object('result', result);
        
        var w = quaternion.w;
        if (Math.abs(w - 1.0) < CesiumMath.EPSILON6) {
            result.x = result.y = result.z = 0;
            return result;
        }

        var scalar = 1.0 / Math.sqrt(1.0 - (w * w));

        result.x = quaternion.x * scalar;
        result.y = quaternion.y * scalar;
        result.z = quaternion.z * scalar;
        return result;
    };

    /**
     * Computes the angle of rotation of the provided quaternion.
     *
     * @param {Quaternion} quaternion The quaternion to use.
     * @returns {Number} The angle of rotation.
     */
    Quaternion.computeAngle = function(quaternion) {
                Check.typeOf.object('quaternion', quaternion);
        
        if (Math.abs(quaternion.w - 1.0) < CesiumMath.EPSILON6) {
            return 0.0;
        }
        return 2.0 * Math.acos(quaternion.w);
    };

    var lerpScratch = new Quaternion();
    /**
     * Computes the linear interpolation or extrapolation at t using the provided quaternions.
     *
     * @param {Quaternion} start The value corresponding to t at 0.0.
     * @param {Quaternion} end The value corresponding to t at 1.0.
     * @param {Number} t The point along t at which to interpolate.
     * @param {Quaternion} result The object onto which to store the result.
     * @returns {Quaternion} The modified result parameter.
     */
    Quaternion.lerp = function(start, end, t, result) {
                Check.typeOf.object('start', start);
        Check.typeOf.object('end', end);
        Check.typeOf.number('t', t);
        Check.typeOf.object('result', result);
        
        lerpScratch = Quaternion.multiplyByScalar(end, t, lerpScratch);
        result = Quaternion.multiplyByScalar(start, 1.0 - t, result);
        return Quaternion.add(lerpScratch, result, result);
    };

    var slerpEndNegated = new Quaternion();
    var slerpScaledP = new Quaternion();
    var slerpScaledR = new Quaternion();
    /**
     * Computes the spherical linear interpolation or extrapolation at t using the provided quaternions.
     *
     * @param {Quaternion} start The value corresponding to t at 0.0.
     * @param {Quaternion} end The value corresponding to t at 1.0.
     * @param {Number} t The point along t at which to interpolate.
     * @param {Quaternion} result The object onto which to store the result.
     * @returns {Quaternion} The modified result parameter.
     *
     * @see Quaternion#fastSlerp
     */
    Quaternion.slerp = function(start, end, t, result) {
                Check.typeOf.object('start', start);
        Check.typeOf.object('end', end);
        Check.typeOf.number('t', t);
        Check.typeOf.object('result', result);
        
        var dot = Quaternion.dot(start, end);

        // The angle between start must be acute. Since q and -q represent
        // the same rotation, negate q to get the acute angle.
        var r = end;
        if (dot < 0.0) {
            dot = -dot;
            r = slerpEndNegated = Quaternion.negate(end, slerpEndNegated);
        }

        // dot > 0, as the dot product approaches 1, the angle between the
        // quaternions vanishes. use linear interpolation.
        if (1.0 - dot < CesiumMath.EPSILON6) {
            return Quaternion.lerp(start, r, t, result);
        }

        var theta = Math.acos(dot);
        slerpScaledP = Quaternion.multiplyByScalar(start, Math.sin((1 - t) * theta), slerpScaledP);
        slerpScaledR = Quaternion.multiplyByScalar(r, Math.sin(t * theta), slerpScaledR);
        result = Quaternion.add(slerpScaledP, slerpScaledR, result);
        return Quaternion.multiplyByScalar(result, 1.0 / Math.sin(theta), result);
    };

    /**
     * The logarithmic quaternion function.
     *
     * @param {Quaternion} quaternion The unit quaternion.
     * @param {Cartesian3} result The object onto which to store the result.
     * @returns {Cartesian3} The modified result parameter.
     */
    Quaternion.log = function(quaternion, result) {
                Check.typeOf.object('quaternion', quaternion);
        Check.typeOf.object('result', result);
        
        var theta = CesiumMath.acosClamped(quaternion.w);
        var thetaOverSinTheta = 0.0;

        if (theta !== 0.0) {
            thetaOverSinTheta = theta / Math.sin(theta);
        }

        return Cartesian3.multiplyByScalar(quaternion, thetaOverSinTheta, result);
    };

    /**
     * The exponential quaternion function.
     *
     * @param {Cartesian3} cartesian The cartesian.
     * @param {Quaternion} result The object onto which to store the result.
     * @returns {Quaternion} The modified result parameter.
     */
    Quaternion.exp = function(cartesian, result) {
                Check.typeOf.object('cartesian', cartesian);
        Check.typeOf.object('result', result);
        
        var theta = Cartesian3.magnitude(cartesian);
        var sinThetaOverTheta = 0.0;

        if (theta !== 0.0) {
            sinThetaOverTheta = Math.sin(theta) / theta;
        }

        result.x = cartesian.x * sinThetaOverTheta;
        result.y = cartesian.y * sinThetaOverTheta;
        result.z = cartesian.z * sinThetaOverTheta;
        result.w = Math.cos(theta);

        return result;
    };

    var squadScratchCartesian0 = new Cartesian3();
    var squadScratchCartesian1 = new Cartesian3();
    var squadScratchQuaternion0 = new Quaternion();
    var squadScratchQuaternion1 = new Quaternion();

    /**
     * Computes an inner quadrangle point.
     * <p>This will compute quaternions that ensure a squad curve is C<sup>1</sup>.</p>
     *
     * @param {Quaternion} q0 The first quaternion.
     * @param {Quaternion} q1 The second quaternion.
     * @param {Quaternion} q2 The third quaternion.
     * @param {Quaternion} result The object onto which to store the result.
     * @returns {Quaternion} The modified result parameter.
     *
     * @see Quaternion#squad
     */
    Quaternion.computeInnerQuadrangle = function(q0, q1, q2, result) {
                Check.typeOf.object('q0', q0);
        Check.typeOf.object('q1', q1);
        Check.typeOf.object('q2', q2);
        Check.typeOf.object('result', result);
        
        var qInv = Quaternion.conjugate(q1, squadScratchQuaternion0);
        Quaternion.multiply(qInv, q2, squadScratchQuaternion1);
        var cart0 = Quaternion.log(squadScratchQuaternion1, squadScratchCartesian0);

        Quaternion.multiply(qInv, q0, squadScratchQuaternion1);
        var cart1 = Quaternion.log(squadScratchQuaternion1, squadScratchCartesian1);

        Cartesian3.add(cart0, cart1, cart0);
        Cartesian3.multiplyByScalar(cart0, 0.25, cart0);
        Cartesian3.negate(cart0, cart0);
        Quaternion.exp(cart0, squadScratchQuaternion0);

        return Quaternion.multiply(q1, squadScratchQuaternion0, result);
    };

    /**
     * Computes the spherical quadrangle interpolation between quaternions.
     *
     * @param {Quaternion} q0 The first quaternion.
     * @param {Quaternion} q1 The second quaternion.
     * @param {Quaternion} s0 The first inner quadrangle.
     * @param {Quaternion} s1 The second inner quadrangle.
     * @param {Number} t The time in [0,1] used to interpolate.
     * @param {Quaternion} result The object onto which to store the result.
     * @returns {Quaternion} The modified result parameter.
     *
     *
     * @example
     * // 1. compute the squad interpolation between two quaternions on a curve
     * var s0 = Cesium.Quaternion.computeInnerQuadrangle(quaternions[i - 1], quaternions[i], quaternions[i + 1], new Cesium.Quaternion());
     * var s1 = Cesium.Quaternion.computeInnerQuadrangle(quaternions[i], quaternions[i + 1], quaternions[i + 2], new Cesium.Quaternion());
     * var q = Cesium.Quaternion.squad(quaternions[i], quaternions[i + 1], s0, s1, t, new Cesium.Quaternion());
     *
     * // 2. compute the squad interpolation as above but where the first quaternion is a end point.
     * var s1 = Cesium.Quaternion.computeInnerQuadrangle(quaternions[0], quaternions[1], quaternions[2], new Cesium.Quaternion());
     * var q = Cesium.Quaternion.squad(quaternions[0], quaternions[1], quaternions[0], s1, t, new Cesium.Quaternion());
     *
     * @see Quaternion#computeInnerQuadrangle
     */
    Quaternion.squad = function(q0, q1, s0, s1, t, result) {
                Check.typeOf.object('q0', q0);
        Check.typeOf.object('q1', q1);
        Check.typeOf.object('s0', s0);
        Check.typeOf.object('s1', s1);
        Check.typeOf.number('t', t);
        Check.typeOf.object('result', result);
        
        var slerp0 = Quaternion.slerp(q0, q1, t, squadScratchQuaternion0);
        var slerp1 = Quaternion.slerp(s0, s1, t, squadScratchQuaternion1);
        return Quaternion.slerp(slerp0, slerp1, 2.0 * t * (1.0 - t), result);
    };

    var fastSlerpScratchQuaternion = new Quaternion();
    var opmu = 1.90110745351730037;
    var u = FeatureDetection.supportsTypedArrays() ? new Float32Array(8) : [];
    var v = FeatureDetection.supportsTypedArrays() ? new Float32Array(8) : [];
    var bT = FeatureDetection.supportsTypedArrays() ? new Float32Array(8) : [];
    var bD = FeatureDetection.supportsTypedArrays() ? new Float32Array(8) : [];

    for (var i = 0; i < 7; ++i) {
        var s = i + 1.0;
        var t = 2.0 * s + 1.0;
        u[i] = 1.0 / (s * t);
        v[i] = s / t;
    }

    u[7] = opmu / (8.0 * 17.0);
    v[7] = opmu * 8.0 / 17.0;

    /**
     * Computes the spherical linear interpolation or extrapolation at t using the provided quaternions.
     * This implementation is faster than {@link Quaternion#slerp}, but is only accurate up to 10<sup>-6</sup>.
     *
     * @param {Quaternion} start The value corresponding to t at 0.0.
     * @param {Quaternion} end The value corresponding to t at 1.0.
     * @param {Number} t The point along t at which to interpolate.
     * @param {Quaternion} result The object onto which to store the result.
     * @returns {Quaternion} The modified result parameter.
     *
     * @see Quaternion#slerp
     */
    Quaternion.fastSlerp = function(start, end, t, result) {
                Check.typeOf.object('start', start);
        Check.typeOf.object('end', end);
        Check.typeOf.number('t', t);
        Check.typeOf.object('result', result);
        
        var x = Quaternion.dot(start, end);

        var sign;
        if (x >= 0) {
            sign = 1.0;
        } else {
            sign = -1.0;
            x = -x;
        }

        var xm1 = x - 1.0;
        var d = 1.0 - t;
        var sqrT = t * t;
        var sqrD = d * d;

        for (var i = 7; i >= 0; --i) {
            bT[i] = (u[i] * sqrT - v[i]) * xm1;
            bD[i] = (u[i] * sqrD - v[i]) * xm1;
        }

        var cT = sign * t * (
            1.0 + bT[0] * (1.0 + bT[1] * (1.0 + bT[2] * (1.0 + bT[3] * (
            1.0 + bT[4] * (1.0 + bT[5] * (1.0 + bT[6] * (1.0 + bT[7]))))))));
        var cD = d * (
            1.0 + bD[0] * (1.0 + bD[1] * (1.0 + bD[2] * (1.0 + bD[3] * (
            1.0 + bD[4] * (1.0 + bD[5] * (1.0 + bD[6] * (1.0 + bD[7]))))))));

        var temp = Quaternion.multiplyByScalar(start, cD, fastSlerpScratchQuaternion);
        Quaternion.multiplyByScalar(end, cT, result);
        return Quaternion.add(temp, result, result);
    };

    /**
     * Computes the spherical quadrangle interpolation between quaternions.
     * An implementation that is faster than {@link Quaternion#squad}, but less accurate.
     *
     * @param {Quaternion} q0 The first quaternion.
     * @param {Quaternion} q1 The second quaternion.
     * @param {Quaternion} s0 The first inner quadrangle.
     * @param {Quaternion} s1 The second inner quadrangle.
     * @param {Number} t The time in [0,1] used to interpolate.
     * @param {Quaternion} result The object onto which to store the result.
     * @returns {Quaternion} The modified result parameter or a new instance if none was provided.
     *
     * @see Quaternion#squad
     */
    Quaternion.fastSquad = function(q0, q1, s0, s1, t, result) {
                Check.typeOf.object('q0', q0);
        Check.typeOf.object('q1', q1);
        Check.typeOf.object('s0', s0);
        Check.typeOf.object('s1', s1);
        Check.typeOf.number('t', t);
        Check.typeOf.object('result', result);
        
        var slerp0 = Quaternion.fastSlerp(q0, q1, t, squadScratchQuaternion0);
        var slerp1 = Quaternion.fastSlerp(s0, s1, t, squadScratchQuaternion1);
        return Quaternion.fastSlerp(slerp0, slerp1, 2.0 * t * (1.0 - t), result);
    };

    /**
     * Compares the provided quaternions componentwise and returns
     * <code>true</code> if they are equal, <code>false</code> otherwise.
     *
     * @param {Quaternion} [left] The first quaternion.
     * @param {Quaternion} [right] The second quaternion.
     * @returns {Boolean} <code>true</code> if left and right are equal, <code>false</code> otherwise.
     */
    Quaternion.equals = function(left, right) {
        return (left === right) ||
               ((defined(left)) &&
                (defined(right)) &&
                (left.x === right.x) &&
                (left.y === right.y) &&
                (left.z === right.z) &&
                (left.w === right.w));
    };

    /**
     * Compares the provided quaternions componentwise and returns
     * <code>true</code> if they are within the provided epsilon,
     * <code>false</code> otherwise.
     *
     * @param {Quaternion} [left] The first quaternion.
     * @param {Quaternion} [right] The second quaternion.
     * @param {Number} epsilon The epsilon to use for equality testing.
     * @returns {Boolean} <code>true</code> if left and right are within the provided epsilon, <code>false</code> otherwise.
     */
    Quaternion.equalsEpsilon = function(left, right, epsilon) {
                Check.typeOf.number('epsilon', epsilon);
        
        return (left === right) ||
               ((defined(left)) &&
                (defined(right)) &&
                (Math.abs(left.x - right.x) <= epsilon) &&
                (Math.abs(left.y - right.y) <= epsilon) &&
                (Math.abs(left.z - right.z) <= epsilon) &&
                (Math.abs(left.w - right.w) <= epsilon));
    };

    /**
     * An immutable Quaternion instance initialized to (0.0, 0.0, 0.0, 0.0).
     *
     * @type {Quaternion}
     * @constant
     */
    Quaternion.ZERO = freezeObject(new Quaternion(0.0, 0.0, 0.0, 0.0));

    /**
     * An immutable Quaternion instance initialized to (0.0, 0.0, 0.0, 1.0).
     *
     * @type {Quaternion}
     * @constant
     */
    Quaternion.IDENTITY = freezeObject(new Quaternion(0.0, 0.0, 0.0, 1.0));

    /**
     * Duplicates this Quaternion instance.
     *
     * @param {Quaternion} [result] The object onto which to store the result.
     * @returns {Quaternion} The modified result parameter or a new Quaternion instance if one was not provided.
     */
    Quaternion.prototype.clone = function(result) {
        return Quaternion.clone(this, result);
    };

    /**
     * Compares this and the provided quaternion componentwise and returns
     * <code>true</code> if they are equal, <code>false</code> otherwise.
     *
     * @param {Quaternion} [right] The right hand side quaternion.
     * @returns {Boolean} <code>true</code> if left and right are equal, <code>false</code> otherwise.
     */
    Quaternion.prototype.equals = function(right) {
        return Quaternion.equals(this, right);
    };

    /**
     * Compares this and the provided quaternion componentwise and returns
     * <code>true</code> if they are within the provided epsilon,
     * <code>false</code> otherwise.
     *
     * @param {Quaternion} [right] The right hand side quaternion.
     * @param {Number} epsilon The epsilon to use for equality testing.
     * @returns {Boolean} <code>true</code> if left and right are within the provided epsilon, <code>false</code> otherwise.
     */
    Quaternion.prototype.equalsEpsilon = function(right, epsilon) {
        return Quaternion.equalsEpsilon(this, right, epsilon);
    };

    /**
     * Returns a string representing this quaternion in the format (x, y, z, w).
     *
     * @returns {String} A string representing this Quaternion.
     */
    Quaternion.prototype.toString = function() {
        return '(' + this.x + ', ' + this.y + ', ' + this.z + ', ' + this.w + ')';
    };

    return Quaternion;
});

define('Core/EllipseGeometryLibrary',[
        './Cartesian3',
        './Math',
        './Matrix3',
        './Quaternion'
    ], function(
        Cartesian3,
        CesiumMath,
        Matrix3,
        Quaternion) {
    'use strict';

    var EllipseGeometryLibrary = {};

    var rotAxis = new Cartesian3();
    var tempVec = new Cartesian3();
    var unitQuat = new Quaternion();
    var rotMtx = new Matrix3();

    function pointOnEllipsoid(theta, rotation, northVec, eastVec, aSqr, ab, bSqr, mag, unitPos, result) {
        var azimuth = theta + rotation;

        Cartesian3.multiplyByScalar(eastVec, Math.cos(azimuth), rotAxis);
        Cartesian3.multiplyByScalar(northVec, Math.sin(azimuth), tempVec);
        Cartesian3.add(rotAxis, tempVec, rotAxis);

        var cosThetaSquared = Math.cos(theta);
        cosThetaSquared = cosThetaSquared * cosThetaSquared;

        var sinThetaSquared = Math.sin(theta);
        sinThetaSquared = sinThetaSquared * sinThetaSquared;

        var radius = ab / Math.sqrt(bSqr * cosThetaSquared + aSqr * sinThetaSquared);
        var angle = radius / mag;

        // Create the quaternion to rotate the position vector to the boundary of the ellipse.
        Quaternion.fromAxisAngle(rotAxis, angle, unitQuat);
        Matrix3.fromQuaternion(unitQuat, rotMtx);

        Matrix3.multiplyByVector(rotMtx, unitPos, result);
        Cartesian3.normalize(result, result);
        Cartesian3.multiplyByScalar(result, mag, result);
        return result;
    }

    var scratchCartesian1 = new Cartesian3();
    var scratchCartesian2 = new Cartesian3();
    var scratchCartesian3 = new Cartesian3();
    var scratchNormal = new Cartesian3();
    /**
     * Returns the positions raised to the given heights
     * @private
     */
    EllipseGeometryLibrary.raisePositionsToHeight = function(positions, options, extrude) {
        var ellipsoid = options.ellipsoid;
        var height = options.height;
        var extrudedHeight = options.extrudedHeight;
        var size = (extrude) ? positions.length / 3 * 2 : positions.length / 3;

        var finalPositions = new Float64Array(size * 3);

        var length = positions.length;
        var bottomOffset = (extrude) ? length : 0;
        for (var i = 0; i < length; i += 3) {
            var i1 = i + 1;
            var i2 = i + 2;

            var position = Cartesian3.fromArray(positions, i, scratchCartesian1);
            ellipsoid.scaleToGeodeticSurface(position, position);

            var extrudedPosition = Cartesian3.clone(position, scratchCartesian2);
            var normal = ellipsoid.geodeticSurfaceNormal(position, scratchNormal);
            var scaledNormal = Cartesian3.multiplyByScalar(normal, height, scratchCartesian3);
            Cartesian3.add(position, scaledNormal, position);

            if (extrude) {
                Cartesian3.multiplyByScalar(normal, extrudedHeight, scaledNormal);
                Cartesian3.add(extrudedPosition, scaledNormal, extrudedPosition);

                finalPositions[i + bottomOffset] = extrudedPosition.x;
                finalPositions[i1 + bottomOffset] = extrudedPosition.y;
                finalPositions[i2 + bottomOffset] = extrudedPosition.z;
            }

            finalPositions[i] = position.x;
            finalPositions[i1] = position.y;
            finalPositions[i2] = position.z;
        }

        return finalPositions;
    };

    var unitPosScratch = new Cartesian3();
    var eastVecScratch = new Cartesian3();
    var northVecScratch = new Cartesian3();
    /**
     * Returns an array of positions that make up the ellipse.
     * @private
     */
    EllipseGeometryLibrary.computeEllipsePositions = function(options, addFillPositions, addEdgePositions) {
        var semiMinorAxis = options.semiMinorAxis;
        var semiMajorAxis = options.semiMajorAxis;
        var rotation = options.rotation;
        var center = options.center;

        // Computing the arc-length of the ellipse is too expensive to be practical. Estimating it using the
        // arc length of the sphere is too inaccurate and creates sharp edges when either the semi-major or
        // semi-minor axis is much bigger than the other. Instead, scale the angle delta to make
        // the distance along the ellipse boundary more closely match the granularity.
        var granularity = options.granularity * 8.0;

        var aSqr = semiMinorAxis * semiMinorAxis;
        var bSqr = semiMajorAxis * semiMajorAxis;
        var ab = semiMajorAxis * semiMinorAxis;

        var mag = Cartesian3.magnitude(center);

        var unitPos = Cartesian3.normalize(center, unitPosScratch);
        var eastVec = Cartesian3.cross(Cartesian3.UNIT_Z, center, eastVecScratch);
        eastVec = Cartesian3.normalize(eastVec, eastVec);
        var northVec = Cartesian3.cross(unitPos, eastVec, northVecScratch);

        // The number of points in the first quadrant
        var numPts = 1 + Math.ceil(CesiumMath.PI_OVER_TWO / granularity);

        var deltaTheta = CesiumMath.PI_OVER_TWO / (numPts - 1);
        var theta = CesiumMath.PI_OVER_TWO - numPts * deltaTheta;
        if (theta < 0.0) {
            numPts -= Math.ceil(Math.abs(theta) / deltaTheta);
        }

        // If the number of points were three, the ellipse
        // would be tessellated like below:
        //
        //         *---*
        //       / | \ | \
        //     *---*---*---*
        //   / | \ | \ | \ | \
        //  / .*---*---*---*. \
        // * ` | \ | \ | \ | `*
        //  \`.*---*---*---*.`/
        //   \ | \ | \ | \ | /
        //     *---*---*---*
        //       \ | \ | /
        //         *---*
        // The first and last column have one position and fan to connect to the adjacent column.
        // Each other vertical column contains an even number of positions.
        var size = 2 * (numPts * (numPts + 2));
        var positions = (addFillPositions) ? new Array(size * 3) : undefined;
        var positionIndex = 0;
        var position = scratchCartesian1;
        var reflectedPosition = scratchCartesian2;

        var outerPositionsLength = (numPts * 4) * 3;
        var outerRightIndex = outerPositionsLength - 1;
        var outerLeftIndex = 0;
        var outerPositions = (addEdgePositions) ? new Array(outerPositionsLength) : undefined;

        var i;
        var j;
        var numInterior;
        var t;
        var interiorPosition;

        // Compute points in the 'eastern' half of the ellipse
        theta = CesiumMath.PI_OVER_TWO;
        position = pointOnEllipsoid(theta, rotation, northVec, eastVec, aSqr, ab, bSqr, mag, unitPos, position);
        if (addFillPositions) {
            positions[positionIndex++] = position.x;
            positions[positionIndex++] = position.y;
            positions[positionIndex++] = position.z;
        }
        if (addEdgePositions) {
            outerPositions[outerRightIndex--] = position.z;
            outerPositions[outerRightIndex--] = position.y;
            outerPositions[outerRightIndex--] = position.x;
        }
        theta = CesiumMath.PI_OVER_TWO -  deltaTheta;
        for (i = 1; i < numPts + 1; ++i) {
            position = pointOnEllipsoid(theta, rotation, northVec, eastVec, aSqr, ab, bSqr, mag, unitPos, position);
            reflectedPosition = pointOnEllipsoid(Math.PI - theta, rotation, northVec, eastVec, aSqr, ab, bSqr, mag, unitPos, reflectedPosition);

            if (addFillPositions) {
                positions[positionIndex++] = position.x;
                positions[positionIndex++] = position.y;
                positions[positionIndex++] = position.z;

                numInterior = 2 * i + 2;
                for (j = 1; j < numInterior - 1; ++j) {
                    t = j / (numInterior - 1);
                    interiorPosition = Cartesian3.lerp(position, reflectedPosition, t, scratchCartesian3);
                    positions[positionIndex++] = interiorPosition.x;
                    positions[positionIndex++] = interiorPosition.y;
                    positions[positionIndex++] = interiorPosition.z;
                }

                positions[positionIndex++] = reflectedPosition.x;
                positions[positionIndex++] = reflectedPosition.y;
                positions[positionIndex++] = reflectedPosition.z;
            }

            if (addEdgePositions) {
                outerPositions[outerRightIndex--] = position.z;
                outerPositions[outerRightIndex--] = position.y;
                outerPositions[outerRightIndex--] = position.x;
                outerPositions[outerLeftIndex++] = reflectedPosition.x;
                outerPositions[outerLeftIndex++] = reflectedPosition.y;
                outerPositions[outerLeftIndex++] = reflectedPosition.z;
            }

            theta = CesiumMath.PI_OVER_TWO - (i + 1) * deltaTheta;
        }

        // Compute points in the 'western' half of the ellipse
        for (i = numPts; i > 1; --i) {
            theta = CesiumMath.PI_OVER_TWO - (i - 1) * deltaTheta;

            position = pointOnEllipsoid(-theta, rotation, northVec, eastVec, aSqr, ab, bSqr, mag, unitPos, position);
            reflectedPosition = pointOnEllipsoid(theta + Math.PI, rotation, northVec, eastVec, aSqr, ab, bSqr, mag, unitPos, reflectedPosition);

            if (addFillPositions) {
                positions[positionIndex++] = position.x;
                positions[positionIndex++] = position.y;
                positions[positionIndex++] = position.z;

                numInterior = 2 * (i - 1) + 2;
                for (j = 1; j < numInterior - 1; ++j) {
                    t = j / (numInterior - 1);
                    interiorPosition = Cartesian3.lerp(position, reflectedPosition, t, scratchCartesian3);
                    positions[positionIndex++] = interiorPosition.x;
                    positions[positionIndex++] = interiorPosition.y;
                    positions[positionIndex++] = interiorPosition.z;
                }

                positions[positionIndex++] = reflectedPosition.x;
                positions[positionIndex++] = reflectedPosition.y;
                positions[positionIndex++] = reflectedPosition.z;
            }

            if (addEdgePositions) {
                outerPositions[outerRightIndex--] = position.z;
                outerPositions[outerRightIndex--] = position.y;
                outerPositions[outerRightIndex--] = position.x;
                outerPositions[outerLeftIndex++] = reflectedPosition.x;
                outerPositions[outerLeftIndex++] = reflectedPosition.y;
                outerPositions[outerLeftIndex++] = reflectedPosition.z;
            }
        }

        theta = CesiumMath.PI_OVER_TWO;
        position = pointOnEllipsoid(-theta, rotation, northVec, eastVec, aSqr, ab, bSqr, mag, unitPos, position);

        var r = {};
        if (addFillPositions) {
            positions[positionIndex++] = position.x;
            positions[positionIndex++] = position.y;
            positions[positionIndex++] = position.z;
            r.positions = positions;
            r.numPts = numPts;
        }
        if (addEdgePositions) {
            outerPositions[outerRightIndex--] = position.z;
            outerPositions[outerRightIndex--] = position.y;
            outerPositions[outerRightIndex--] = position.x;
            r.outerPositions = outerPositions;
        }

        return r;
    };

    return EllipseGeometryLibrary;
});

define('Core/Cartesian2',[
        './Check',
        './defaultValue',
        './defined',
        './DeveloperError',
        './freezeObject',
        './Math'
    ], function(
        Check,
        defaultValue,
        defined,
        DeveloperError,
        freezeObject,
        CesiumMath) {
    'use strict';

    /**
     * A 2D Cartesian point.
     * @alias Cartesian2
     * @constructor
     *
     * @param {Number} [x=0.0] The X component.
     * @param {Number} [y=0.0] The Y component.
     *
     * @see Cartesian3
     * @see Cartesian4
     * @see Packable
     */
    function Cartesian2(x, y) {
        /**
         * The X component.
         * @type {Number}
         * @default 0.0
         */
        this.x = defaultValue(x, 0.0);

        /**
         * The Y component.
         * @type {Number}
         * @default 0.0
         */
        this.y = defaultValue(y, 0.0);
    }

    /**
     * Creates a Cartesian2 instance from x and y coordinates.
     *
     * @param {Number} x The x coordinate.
     * @param {Number} y The y coordinate.
     * @param {Cartesian2} [result] The object onto which to store the result.
     * @returns {Cartesian2} The modified result parameter or a new Cartesian2 instance if one was not provided.
     */
    Cartesian2.fromElements = function(x, y, result) {
        if (!defined(result)) {
            return new Cartesian2(x, y);
        }

        result.x = x;
        result.y = y;
        return result;
    };

    /**
     * Duplicates a Cartesian2 instance.
     *
     * @param {Cartesian2} cartesian The Cartesian to duplicate.
     * @param {Cartesian2} [result] The object onto which to store the result.
     * @returns {Cartesian2} The modified result parameter or a new Cartesian2 instance if one was not provided. (Returns undefined if cartesian is undefined)
     */
    Cartesian2.clone = function(cartesian, result) {
        if (!defined(cartesian)) {
            return undefined;
        }
        if (!defined(result)) {
            return new Cartesian2(cartesian.x, cartesian.y);
        }

        result.x = cartesian.x;
        result.y = cartesian.y;
        return result;
    };

    /**
     * Creates a Cartesian2 instance from an existing Cartesian3.  This simply takes the
     * x and y properties of the Cartesian3 and drops z.
     * @function
     *
     * @param {Cartesian3} cartesian The Cartesian3 instance to create a Cartesian2 instance from.
     * @param {Cartesian2} [result] The object onto which to store the result.
     * @returns {Cartesian2} The modified result parameter or a new Cartesian2 instance if one was not provided.
     */
    Cartesian2.fromCartesian3 = Cartesian2.clone;

    /**
     * Creates a Cartesian2 instance from an existing Cartesian4.  This simply takes the
     * x and y properties of the Cartesian4 and drops z and w.
     * @function
     *
     * @param {Cartesian4} cartesian The Cartesian4 instance to create a Cartesian2 instance from.
     * @param {Cartesian2} [result] The object onto which to store the result.
     * @returns {Cartesian2} The modified result parameter or a new Cartesian2 instance if one was not provided.
     */
    Cartesian2.fromCartesian4 = Cartesian2.clone;

    /**
     * The number of elements used to pack the object into an array.
     * @type {Number}
     */
    Cartesian2.packedLength = 2;

    /**
     * Stores the provided instance into the provided array.
     *
     * @param {Cartesian2} value The value to pack.
     * @param {Number[]} array The array to pack into.
     * @param {Number} [startingIndex=0] The index into the array at which to start packing the elements.
     *
     * @returns {Number[]} The array that was packed into
     */
    Cartesian2.pack = function(value, array, startingIndex) {
                Check.typeOf.object('value', value);
        Check.defined('array', array);
        
        startingIndex = defaultValue(startingIndex, 0);

        array[startingIndex++] = value.x;
        array[startingIndex] = value.y;

        return array;
    };

    /**
     * Retrieves an instance from a packed array.
     *
     * @param {Number[]} array The packed array.
     * @param {Number} [startingIndex=0] The starting index of the element to be unpacked.
     * @param {Cartesian2} [result] The object into which to store the result.
     * @returns {Cartesian2} The modified result parameter or a new Cartesian2 instance if one was not provided.
     */
    Cartesian2.unpack = function(array, startingIndex, result) {
                Check.defined('array', array);
        
        startingIndex = defaultValue(startingIndex, 0);

        if (!defined(result)) {
            result = new Cartesian2();
        }
        result.x = array[startingIndex++];
        result.y = array[startingIndex];
        return result;
    };

    /**
     * Flattens an array of Cartesian2s into and array of components.
     *
     * @param {Cartesian2[]} array The array of cartesians to pack.
     * @param {Number[]} [result] The array onto which to store the result.
     * @returns {Number[]} The packed array.
     */
    Cartesian2.packArray = function(array, result) {
                Check.defined('array', array);
        
        var length = array.length;
        if (!defined(result)) {
            result = new Array(length * 2);
        } else {
            result.length = length * 2;
        }

        for (var i = 0; i < length; ++i) {
            Cartesian2.pack(array[i], result, i * 2);
        }
        return result;
    };

    /**
     * Unpacks an array of cartesian components into and array of Cartesian2s.
     *
     * @param {Number[]} array The array of components to unpack.
     * @param {Cartesian2[]} [result] The array onto which to store the result.
     * @returns {Cartesian2[]} The unpacked array.
     */
    Cartesian2.unpackArray = function(array, result) {
                Check.defined('array', array);
        
        var length = array.length;
        if (!defined(result)) {
            result = new Array(length / 2);
        } else {
            result.length = length / 2;
        }

        for (var i = 0; i < length; i += 2) {
            var index = i / 2;
            result[index] = Cartesian2.unpack(array, i, result[index]);
        }
        return result;
    };

    /**
     * Creates a Cartesian2 from two consecutive elements in an array.
     * @function
     *
     * @param {Number[]} array The array whose two consecutive elements correspond to the x and y components, respectively.
     * @param {Number} [startingIndex=0] The offset into the array of the first element, which corresponds to the x component.
     * @param {Cartesian2} [result] The object onto which to store the result.
     * @returns {Cartesian2} The modified result parameter or a new Cartesian2 instance if one was not provided.
     *
     * @example
     * // Create a Cartesian2 with (1.0, 2.0)
     * var v = [1.0, 2.0];
     * var p = Cesium.Cartesian2.fromArray(v);
     *
     * // Create a Cartesian2 with (1.0, 2.0) using an offset into an array
     * var v2 = [0.0, 0.0, 1.0, 2.0];
     * var p2 = Cesium.Cartesian2.fromArray(v2, 2);
     */
    Cartesian2.fromArray = Cartesian2.unpack;

    /**
     * Computes the value of the maximum component for the supplied Cartesian.
     *
     * @param {Cartesian2} cartesian The cartesian to use.
     * @returns {Number} The value of the maximum component.
     */
    Cartesian2.maximumComponent = function(cartesian) {
                Check.typeOf.object('cartesian', cartesian);
        
        return Math.max(cartesian.x, cartesian.y);
    };

    /**
     * Computes the value of the minimum component for the supplied Cartesian.
     *
     * @param {Cartesian2} cartesian The cartesian to use.
     * @returns {Number} The value of the minimum component.
     */
    Cartesian2.minimumComponent = function(cartesian) {
                Check.typeOf.object('cartesian', cartesian);
        
        return Math.min(cartesian.x, cartesian.y);
    };

    /**
     * Compares two Cartesians and computes a Cartesian which contains the minimum components of the supplied Cartesians.
     *
     * @param {Cartesian2} first A cartesian to compare.
     * @param {Cartesian2} second A cartesian to compare.
     * @param {Cartesian2} result The object into which to store the result.
     * @returns {Cartesian2} A cartesian with the minimum components.
     */
    Cartesian2.minimumByComponent = function(first, second, result) {
                Check.typeOf.object('first', first);
        Check.typeOf.object('second', second);
        Check.typeOf.object('result', result);
        
        result.x = Math.min(first.x, second.x);
        result.y = Math.min(first.y, second.y);

        return result;
    };

    /**
     * Compares two Cartesians and computes a Cartesian which contains the maximum components of the supplied Cartesians.
     *
     * @param {Cartesian2} first A cartesian to compare.
     * @param {Cartesian2} second A cartesian to compare.
     * @param {Cartesian2} result The object into which to store the result.
     * @returns {Cartesian2} A cartesian with the maximum components.
     */
    Cartesian2.maximumByComponent = function(first, second, result) {
                Check.typeOf.object('first', first);
        Check.typeOf.object('second', second);
        Check.typeOf.object('result', result);
        
        result.x = Math.max(first.x, second.x);
        result.y = Math.max(first.y, second.y);
        return result;
    };

    /**
     * Computes the provided Cartesian's squared magnitude.
     *
     * @param {Cartesian2} cartesian The Cartesian instance whose squared magnitude is to be computed.
     * @returns {Number} The squared magnitude.
     */
    Cartesian2.magnitudeSquared = function(cartesian) {
                Check.typeOf.object('cartesian', cartesian);
        
        return cartesian.x * cartesian.x + cartesian.y * cartesian.y;
    };

    /**
     * Computes the Cartesian's magnitude (length).
     *
     * @param {Cartesian2} cartesian The Cartesian instance whose magnitude is to be computed.
     * @returns {Number} The magnitude.
     */
    Cartesian2.magnitude = function(cartesian) {
        return Math.sqrt(Cartesian2.magnitudeSquared(cartesian));
    };

    var distanceScratch = new Cartesian2();

    /**
     * Computes the distance between two points.
     *
     * @param {Cartesian2} left The first point to compute the distance from.
     * @param {Cartesian2} right The second point to compute the distance to.
     * @returns {Number} The distance between two points.
     *
     * @example
     * // Returns 1.0
     * var d = Cesium.Cartesian2.distance(new Cesium.Cartesian2(1.0, 0.0), new Cesium.Cartesian2(2.0, 0.0));
     */
    Cartesian2.distance = function(left, right) {
                Check.typeOf.object('left', left);
        Check.typeOf.object('right', right);
        
        Cartesian2.subtract(left, right, distanceScratch);
        return Cartesian2.magnitude(distanceScratch);
    };

    /**
     * Computes the squared distance between two points.  Comparing squared distances
     * using this function is more efficient than comparing distances using {@link Cartesian2#distance}.
     *
     * @param {Cartesian2} left The first point to compute the distance from.
     * @param {Cartesian2} right The second point to compute the distance to.
     * @returns {Number} The distance between two points.
     *
     * @example
     * // Returns 4.0, not 2.0
     * var d = Cesium.Cartesian2.distance(new Cesium.Cartesian2(1.0, 0.0), new Cesium.Cartesian2(3.0, 0.0));
     */
    Cartesian2.distanceSquared = function(left, right) {
                Check.typeOf.object('left', left);
        Check.typeOf.object('right', right);
        
        Cartesian2.subtract(left, right, distanceScratch);
        return Cartesian2.magnitudeSquared(distanceScratch);
    };

    /**
     * Computes the normalized form of the supplied Cartesian.
     *
     * @param {Cartesian2} cartesian The Cartesian to be normalized.
     * @param {Cartesian2} result The object onto which to store the result.
     * @returns {Cartesian2} The modified result parameter.
     */
    Cartesian2.normalize = function(cartesian, result) {
                Check.typeOf.object('cartesian', cartesian);
        Check.typeOf.object('result', result);
        
        var magnitude = Cartesian2.magnitude(cartesian);

        result.x = cartesian.x / magnitude;
        result.y = cartesian.y / magnitude;

                if (isNaN(result.x) || isNaN(result.y)) {
            throw new DeveloperError('normalized result is not a number');
        }
        
        return result;
    };

    /**
     * Computes the dot (scalar) product of two Cartesians.
     *
     * @param {Cartesian2} left The first Cartesian.
     * @param {Cartesian2} right The second Cartesian.
     * @returns {Number} The dot product.
     */
    Cartesian2.dot = function(left, right) {
                Check.typeOf.object('left', left);
        Check.typeOf.object('right', right);
        
        return left.x * right.x + left.y * right.y;
    };

    /**
     * Computes the componentwise product of two Cartesians.
     *
     * @param {Cartesian2} left The first Cartesian.
     * @param {Cartesian2} right The second Cartesian.
     * @param {Cartesian2} result The object onto which to store the result.
     * @returns {Cartesian2} The modified result parameter.
     */
    Cartesian2.multiplyComponents = function(left, right, result) {
                Check.typeOf.object('left', left);
        Check.typeOf.object('right', right);
        Check.typeOf.object('result', result);
        
        result.x = left.x * right.x;
        result.y = left.y * right.y;
        return result;
    };

    /**
     * Computes the componentwise quotient of two Cartesians.
     *
     * @param {Cartesian2} left The first Cartesian.
     * @param {Cartesian2} right The second Cartesian.
     * @param {Cartesian2} result The object onto which to store the result.
     * @returns {Cartesian2} The modified result parameter.
     */
    Cartesian2.divideComponents = function(left, right, result) {
                Check.typeOf.object('left', left);
        Check.typeOf.object('right', right);
        Check.typeOf.object('result', result);
        
        result.x = left.x / right.x;
        result.y = left.y / right.y;
        return result;
    };

    /**
     * Computes the componentwise sum of two Cartesians.
     *
     * @param {Cartesian2} left The first Cartesian.
     * @param {Cartesian2} right The second Cartesian.
     * @param {Cartesian2} result The object onto which to store the result.
     * @returns {Cartesian2} The modified result parameter.
     */
    Cartesian2.add = function(left, right, result) {
                Check.typeOf.object('left', left);
        Check.typeOf.object('right', right);
        Check.typeOf.object('result', result);
        
        result.x = left.x + right.x;
        result.y = left.y + right.y;
        return result;
    };

    /**
     * Computes the componentwise difference of two Cartesians.
     *
     * @param {Cartesian2} left The first Cartesian.
     * @param {Cartesian2} right The second Cartesian.
     * @param {Cartesian2} result The object onto which to store the result.
     * @returns {Cartesian2} The modified result parameter.
     */
    Cartesian2.subtract = function(left, right, result) {
                Check.typeOf.object('left', left);
        Check.typeOf.object('right', right);
        Check.typeOf.object('result', result);
        
        result.x = left.x - right.x;
        result.y = left.y - right.y;
        return result;
    };

    /**
     * Multiplies the provided Cartesian componentwise by the provided scalar.
     *
     * @param {Cartesian2} cartesian The Cartesian to be scaled.
     * @param {Number} scalar The scalar to multiply with.
     * @param {Cartesian2} result The object onto which to store the result.
     * @returns {Cartesian2} The modified result parameter.
     */
    Cartesian2.multiplyByScalar = function(cartesian, scalar, result) {
                Check.typeOf.object('cartesian', cartesian);
        Check.typeOf.number('scalar', scalar);
        Check.typeOf.object('result', result);
        
        result.x = cartesian.x * scalar;
        result.y = cartesian.y * scalar;
        return result;
    };

    /**
     * Divides the provided Cartesian componentwise by the provided scalar.
     *
     * @param {Cartesian2} cartesian The Cartesian to be divided.
     * @param {Number} scalar The scalar to divide by.
     * @param {Cartesian2} result The object onto which to store the result.
     * @returns {Cartesian2} The modified result parameter.
     */
    Cartesian2.divideByScalar = function(cartesian, scalar, result) {
                Check.typeOf.object('cartesian', cartesian);
        Check.typeOf.number('scalar', scalar);
        Check.typeOf.object('result', result);
        
        result.x = cartesian.x / scalar;
        result.y = cartesian.y / scalar;
        return result;
    };

    /**
     * Negates the provided Cartesian.
     *
     * @param {Cartesian2} cartesian The Cartesian to be negated.
     * @param {Cartesian2} result The object onto which to store the result.
     * @returns {Cartesian2} The modified result parameter.
     */
    Cartesian2.negate = function(cartesian, result) {
                Check.typeOf.object('cartesian', cartesian);
        Check.typeOf.object('result', result);
        
        result.x = -cartesian.x;
        result.y = -cartesian.y;
        return result;
    };

    /**
     * Computes the absolute value of the provided Cartesian.
     *
     * @param {Cartesian2} cartesian The Cartesian whose absolute value is to be computed.
     * @param {Cartesian2} result The object onto which to store the result.
     * @returns {Cartesian2} The modified result parameter.
     */
    Cartesian2.abs = function(cartesian, result) {
                Check.typeOf.object('cartesian', cartesian);
        Check.typeOf.object('result', result);
        
        result.x = Math.abs(cartesian.x);
        result.y = Math.abs(cartesian.y);
        return result;
    };

    var lerpScratch = new Cartesian2();
    /**
     * Computes the linear interpolation or extrapolation at t using the provided cartesians.
     *
     * @param {Cartesian2} start The value corresponding to t at 0.0.
     * @param {Cartesian2} end The value corresponding to t at 1.0.
     * @param {Number} t The point along t at which to interpolate.
     * @param {Cartesian2} result The object onto which to store the result.
     * @returns {Cartesian2} The modified result parameter.
     */
    Cartesian2.lerp = function(start, end, t, result) {
                Check.typeOf.object('start', start);
        Check.typeOf.object('end', end);
        Check.typeOf.number('t', t);
        Check.typeOf.object('result', result);
        
        Cartesian2.multiplyByScalar(end, t, lerpScratch);
        result = Cartesian2.multiplyByScalar(start, 1.0 - t, result);
        return Cartesian2.add(lerpScratch, result, result);
    };

    var angleBetweenScratch = new Cartesian2();
    var angleBetweenScratch2 = new Cartesian2();
    /**
     * Returns the angle, in radians, between the provided Cartesians.
     *
     * @param {Cartesian2} left The first Cartesian.
     * @param {Cartesian2} right The second Cartesian.
     * @returns {Number} The angle between the Cartesians.
     */
    Cartesian2.angleBetween = function(left, right) {
                Check.typeOf.object('left', left);
        Check.typeOf.object('right', right);
        
        Cartesian2.normalize(left, angleBetweenScratch);
        Cartesian2.normalize(right, angleBetweenScratch2);
        return CesiumMath.acosClamped(Cartesian2.dot(angleBetweenScratch, angleBetweenScratch2));
    };

    var mostOrthogonalAxisScratch = new Cartesian2();
    /**
     * Returns the axis that is most orthogonal to the provided Cartesian.
     *
     * @param {Cartesian2} cartesian The Cartesian on which to find the most orthogonal axis.
     * @param {Cartesian2} result The object onto which to store the result.
     * @returns {Cartesian2} The most orthogonal axis.
     */
    Cartesian2.mostOrthogonalAxis = function(cartesian, result) {
                Check.typeOf.object('cartesian', cartesian);
        Check.typeOf.object('result', result);
        
        var f = Cartesian2.normalize(cartesian, mostOrthogonalAxisScratch);
        Cartesian2.abs(f, f);

        if (f.x <= f.y) {
            result = Cartesian2.clone(Cartesian2.UNIT_X, result);
        } else {
            result = Cartesian2.clone(Cartesian2.UNIT_Y, result);
        }

        return result;
    };

    /**
     * Compares the provided Cartesians componentwise and returns
     * <code>true</code> if they are equal, <code>false</code> otherwise.
     *
     * @param {Cartesian2} [left] The first Cartesian.
     * @param {Cartesian2} [right] The second Cartesian.
     * @returns {Boolean} <code>true</code> if left and right are equal, <code>false</code> otherwise.
     */
    Cartesian2.equals = function(left, right) {
        return (left === right) ||
               ((defined(left)) &&
                (defined(right)) &&
                (left.x === right.x) &&
                (left.y === right.y));
    };

    /**
     * @private
     */
    Cartesian2.equalsArray = function(cartesian, array, offset) {
        return cartesian.x === array[offset] &&
               cartesian.y === array[offset + 1];
    };

    /**
     * Compares the provided Cartesians componentwise and returns
     * <code>true</code> if they pass an absolute or relative tolerance test,
     * <code>false</code> otherwise.
     *
     * @param {Cartesian2} [left] The first Cartesian.
     * @param {Cartesian2} [right] The second Cartesian.
     * @param {Number} relativeEpsilon The relative epsilon tolerance to use for equality testing.
     * @param {Number} [absoluteEpsilon=relativeEpsilon] The absolute epsilon tolerance to use for equality testing.
     * @returns {Boolean} <code>true</code> if left and right are within the provided epsilon, <code>false</code> otherwise.
     */
    Cartesian2.equalsEpsilon = function(left, right, relativeEpsilon, absoluteEpsilon) {
        return (left === right) ||
               (defined(left) &&
                defined(right) &&
                CesiumMath.equalsEpsilon(left.x, right.x, relativeEpsilon, absoluteEpsilon) &&
                CesiumMath.equalsEpsilon(left.y, right.y, relativeEpsilon, absoluteEpsilon));
    };

    /**
     * An immutable Cartesian2 instance initialized to (0.0, 0.0).
     *
     * @type {Cartesian2}
     * @constant
     */
    Cartesian2.ZERO = freezeObject(new Cartesian2(0.0, 0.0));

    /**
     * An immutable Cartesian2 instance initialized to (1.0, 0.0).
     *
     * @type {Cartesian2}
     * @constant
     */
    Cartesian2.UNIT_X = freezeObject(new Cartesian2(1.0, 0.0));

    /**
     * An immutable Cartesian2 instance initialized to (0.0, 1.0).
     *
     * @type {Cartesian2}
     * @constant
     */
    Cartesian2.UNIT_Y = freezeObject(new Cartesian2(0.0, 1.0));

    /**
     * Duplicates this Cartesian2 instance.
     *
     * @param {Cartesian2} [result] The object onto which to store the result.
     * @returns {Cartesian2} The modified result parameter or a new Cartesian2 instance if one was not provided.
     */
    Cartesian2.prototype.clone = function(result) {
        return Cartesian2.clone(this, result);
    };

    /**
     * Compares this Cartesian against the provided Cartesian componentwise and returns
     * <code>true</code> if they are equal, <code>false</code> otherwise.
     *
     * @param {Cartesian2} [right] The right hand side Cartesian.
     * @returns {Boolean} <code>true</code> if they are equal, <code>false</code> otherwise.
     */
    Cartesian2.prototype.equals = function(right) {
        return Cartesian2.equals(this, right);
    };

    /**
     * Compares this Cartesian against the provided Cartesian componentwise and returns
     * <code>true</code> if they pass an absolute or relative tolerance test,
     * <code>false</code> otherwise.
     *
     * @param {Cartesian2} [right] The right hand side Cartesian.
     * @param {Number} relativeEpsilon The relative epsilon tolerance to use for equality testing.
     * @param {Number} [absoluteEpsilon=relativeEpsilon] The absolute epsilon tolerance to use for equality testing.
     * @returns {Boolean} <code>true</code> if they are within the provided epsilon, <code>false</code> otherwise.
     */
    Cartesian2.prototype.equalsEpsilon = function(right, relativeEpsilon, absoluteEpsilon) {
        return Cartesian2.equalsEpsilon(this, right, relativeEpsilon, absoluteEpsilon);
    };

    /**
     * Creates a string representing this Cartesian in the format '(x, y)'.
     *
     * @returns {String} A string representing the provided Cartesian in the format '(x, y)'.
     */
    Cartesian2.prototype.toString = function() {
        return '(' + this.x + ', ' + this.y + ')';
    };

    return Cartesian2;
});

define('Core/GeometryOffsetAttribute',[
        '../Core/freezeObject'
    ], function(
        freezeObject) {
    'use strict';

    /**
     * Represents which vertices should have a value of `true` for the `applyOffset` attribute
     * @private
     */
    var GeometryOffsetAttribute = {
        NONE : 0,
        TOP : 1,
        ALL : 2
    };

    return freezeObject(GeometryOffsetAttribute);
});

define('Core/GeometryType',[
        './freezeObject'
    ], function(
        freezeObject) {
    'use strict';

    /**
     * @private
     */
    var GeometryType = {
        NONE : 0,
        TRIANGLES : 1,
        LINES : 2,
        POLYLINES : 3
    };

    return freezeObject(GeometryType);
});

define('Core/Matrix2',[
        './Cartesian2',
        './Check',
        './defaultValue',
        './defined',
        './defineProperties',
        './freezeObject'
    ], function(
        Cartesian2,
        Check,
        defaultValue,
        defined,
        defineProperties,
        freezeObject) {
    'use strict';

    /**
     * A 2x2 matrix, indexable as a column-major order array.
     * Constructor parameters are in row-major order for code readability.
     * @alias Matrix2
     * @constructor
     *
     * @param {Number} [column0Row0=0.0] The value for column 0, row 0.
     * @param {Number} [column1Row0=0.0] The value for column 1, row 0.
     * @param {Number} [column0Row1=0.0] The value for column 0, row 1.
     * @param {Number} [column1Row1=0.0] The value for column 1, row 1.
     *
     * @see Matrix2.fromColumnMajorArray
     * @see Matrix2.fromRowMajorArray
     * @see Matrix2.fromScale
     * @see Matrix2.fromUniformScale
     * @see Matrix3
     * @see Matrix4
     */
    function Matrix2(column0Row0, column1Row0, column0Row1, column1Row1) {
        this[0] = defaultValue(column0Row0, 0.0);
        this[1] = defaultValue(column0Row1, 0.0);
        this[2] = defaultValue(column1Row0, 0.0);
        this[3] = defaultValue(column1Row1, 0.0);
    }

    /**
     * The number of elements used to pack the object into an array.
     * @type {Number}
     */
    Matrix2.packedLength = 4;

    /**
     * Stores the provided instance into the provided array.
     *
     * @param {Matrix2} value The value to pack.
     * @param {Number[]} array The array to pack into.
     * @param {Number} [startingIndex=0] The index into the array at which to start packing the elements.
     *
     * @returns {Number[]} The array that was packed into
     */
    Matrix2.pack = function(value, array, startingIndex) {
                Check.typeOf.object('value', value);
        Check.defined('array', array);
        
        startingIndex = defaultValue(startingIndex, 0);

        array[startingIndex++] = value[0];
        array[startingIndex++] = value[1];
        array[startingIndex++] = value[2];
        array[startingIndex++] = value[3];

        return array;
    };

    /**
     * Retrieves an instance from a packed array.
     *
     * @param {Number[]} array The packed array.
     * @param {Number} [startingIndex=0] The starting index of the element to be unpacked.
     * @param {Matrix2} [result] The object into which to store the result.
     * @returns {Matrix2} The modified result parameter or a new Matrix2 instance if one was not provided.
     */
    Matrix2.unpack = function(array, startingIndex, result) {
                Check.defined('array', array);
        
        startingIndex = defaultValue(startingIndex, 0);

        if (!defined(result)) {
            result = new Matrix2();
        }

        result[0] = array[startingIndex++];
        result[1] = array[startingIndex++];
        result[2] = array[startingIndex++];
        result[3] = array[startingIndex++];
        return result;
    };

    /**
     * Duplicates a Matrix2 instance.
     *
     * @param {Matrix2} matrix The matrix to duplicate.
     * @param {Matrix2} [result] The object onto which to store the result.
     * @returns {Matrix2} The modified result parameter or a new Matrix2 instance if one was not provided. (Returns undefined if matrix is undefined)
     */
    Matrix2.clone = function(matrix, result) {
        if (!defined(matrix)) {
            return undefined;
        }
        if (!defined(result)) {
            return new Matrix2(matrix[0], matrix[2],
                               matrix[1], matrix[3]);
        }
        result[0] = matrix[0];
        result[1] = matrix[1];
        result[2] = matrix[2];
        result[3] = matrix[3];
        return result;
    };

    /**
     * Creates a Matrix2 from 4 consecutive elements in an array.
     *
     * @param {Number[]} array The array whose 4 consecutive elements correspond to the positions of the matrix.  Assumes column-major order.
     * @param {Number} [startingIndex=0] The offset into the array of the first element, which corresponds to first column first row position in the matrix.
     * @param {Matrix2} [result] The object onto which to store the result.
     * @returns {Matrix2} The modified result parameter or a new Matrix2 instance if one was not provided.
     *
     * @example
     * // Create the Matrix2:
     * // [1.0, 2.0]
     * // [1.0, 2.0]
     *
     * var v = [1.0, 1.0, 2.0, 2.0];
     * var m = Cesium.Matrix2.fromArray(v);
     *
     * // Create same Matrix2 with using an offset into an array
     * var v2 = [0.0, 0.0, 1.0, 1.0, 2.0, 2.0];
     * var m2 = Cesium.Matrix2.fromArray(v2, 2);
     */
    Matrix2.fromArray = function(array, startingIndex, result) {
                Check.defined('array', array);
        
        startingIndex = defaultValue(startingIndex, 0);

        if (!defined(result)) {
            result = new Matrix2();
        }

        result[0] = array[startingIndex];
        result[1] = array[startingIndex + 1];
        result[2] = array[startingIndex + 2];
        result[3] = array[startingIndex + 3];
        return result;
    };

    /**
     * Creates a Matrix2 instance from a column-major order array.
     *
     * @param {Number[]} values The column-major order array.
     * @param {Matrix2} [result] The object in which the result will be stored, if undefined a new instance will be created.
     * @returns {Matrix2} The modified result parameter, or a new Matrix2 instance if one was not provided.
     */
    Matrix2.fromColumnMajorArray = function(values, result) {
                Check.defined('values', values);
        
        return Matrix2.clone(values, result);
    };

    /**
     * Creates a Matrix2 instance from a row-major order array.
     * The resulting matrix will be in column-major order.
     *
     * @param {Number[]} values The row-major order array.
     * @param {Matrix2} [result] The object in which the result will be stored, if undefined a new instance will be created.
     * @returns {Matrix2} The modified result parameter, or a new Matrix2 instance if one was not provided.
     */
    Matrix2.fromRowMajorArray = function(values, result) {
                Check.defined('values', values);
        
        if (!defined(result)) {
            return new Matrix2(values[0], values[1],
                               values[2], values[3]);
        }
        result[0] = values[0];
        result[1] = values[2];
        result[2] = values[1];
        result[3] = values[3];
        return result;
    };

    /**
     * Computes a Matrix2 instance representing a non-uniform scale.
     *
     * @param {Cartesian2} scale The x and y scale factors.
     * @param {Matrix2} [result] The object in which the result will be stored, if undefined a new instance will be created.
     * @returns {Matrix2} The modified result parameter, or a new Matrix2 instance if one was not provided.
     *
     * @example
     * // Creates
     * //   [7.0, 0.0]
     * //   [0.0, 8.0]
     * var m = Cesium.Matrix2.fromScale(new Cesium.Cartesian2(7.0, 8.0));
     */
    Matrix2.fromScale = function(scale, result) {
                Check.typeOf.object('scale', scale);
        
        if (!defined(result)) {
            return new Matrix2(
                scale.x, 0.0,
                0.0,     scale.y);
        }

        result[0] = scale.x;
        result[1] = 0.0;
        result[2] = 0.0;
        result[3] = scale.y;
        return result;
    };

    /**
     * Computes a Matrix2 instance representing a uniform scale.
     *
     * @param {Number} scale The uniform scale factor.
     * @param {Matrix2} [result] The object in which the result will be stored, if undefined a new instance will be created.
     * @returns {Matrix2} The modified result parameter, or a new Matrix2 instance if one was not provided.
     *
     * @example
     * // Creates
     * //   [2.0, 0.0]
     * //   [0.0, 2.0]
     * var m = Cesium.Matrix2.fromUniformScale(2.0);
     */
    Matrix2.fromUniformScale = function(scale, result) {
                Check.typeOf.number('scale', scale);
        
        if (!defined(result)) {
            return new Matrix2(
                scale, 0.0,
                0.0,   scale);
        }

        result[0] = scale;
        result[1] = 0.0;
        result[2] = 0.0;
        result[3] = scale;
        return result;
    };

    /**
     * Creates a rotation matrix.
     *
     * @param {Number} angle The angle, in radians, of the rotation.  Positive angles are counterclockwise.
     * @param {Matrix2} [result] The object in which the result will be stored, if undefined a new instance will be created.
     * @returns {Matrix2} The modified result parameter, or a new Matrix2 instance if one was not provided.
     *
     * @example
     * // Rotate a point 45 degrees counterclockwise.
     * var p = new Cesium.Cartesian2(5, 6);
     * var m = Cesium.Matrix2.fromRotation(Cesium.Math.toRadians(45.0));
     * var rotated = Cesium.Matrix2.multiplyByVector(m, p, new Cesium.Cartesian2());
     */
    Matrix2.fromRotation = function(angle, result) {
                Check.typeOf.number('angle', angle);
        
        var cosAngle = Math.cos(angle);
        var sinAngle = Math.sin(angle);

        if (!defined(result)) {
            return new Matrix2(
                cosAngle, -sinAngle,
                sinAngle, cosAngle);
        }
        result[0] = cosAngle;
        result[1] = sinAngle;
        result[2] = -sinAngle;
        result[3] = cosAngle;
        return result;
    };

    /**
     * Creates an Array from the provided Matrix2 instance.
     * The array will be in column-major order.
     *
     * @param {Matrix2} matrix The matrix to use..
     * @param {Number[]} [result] The Array onto which to store the result.
     * @returns {Number[]} The modified Array parameter or a new Array instance if one was not provided.
     */
    Matrix2.toArray = function(matrix, result) {
                Check.typeOf.object('matrix', matrix);
        
        if (!defined(result)) {
            return [matrix[0], matrix[1], matrix[2], matrix[3]];
        }
        result[0] = matrix[0];
        result[1] = matrix[1];
        result[2] = matrix[2];
        result[3] = matrix[3];
        return result;
    };

    /**
     * Computes the array index of the element at the provided row and column.
     *
     * @param {Number} row The zero-based index of the row.
     * @param {Number} column The zero-based index of the column.
     * @returns {Number} The index of the element at the provided row and column.
     *
     * @exception {DeveloperError} row must be 0 or 1.
     * @exception {DeveloperError} column must be 0 or 1.
     *
     * @example
     * var myMatrix = new Cesium.Matrix2();
     * var column1Row0Index = Cesium.Matrix2.getElementIndex(1, 0);
     * var column1Row0 = myMatrix[column1Row0Index]
     * myMatrix[column1Row0Index] = 10.0;
     */
    Matrix2.getElementIndex = function(column, row) {
                Check.typeOf.number.greaterThanOrEquals('row', row, 0);
        Check.typeOf.number.lessThanOrEquals('row', row, 1);

        Check.typeOf.number.greaterThanOrEquals('column', column, 0);
        Check.typeOf.number.lessThanOrEquals('column', column, 1);
        
        return column * 2 + row;
    };

    /**
     * Retrieves a copy of the matrix column at the provided index as a Cartesian2 instance.
     *
     * @param {Matrix2} matrix The matrix to use.
     * @param {Number} index The zero-based index of the column to retrieve.
     * @param {Cartesian2} result The object onto which to store the result.
     * @returns {Cartesian2} The modified result parameter.
     *
     * @exception {DeveloperError} index must be 0 or 1.
     */
    Matrix2.getColumn = function(matrix, index, result) {
                Check.typeOf.object('matrix', matrix);

        Check.typeOf.number.greaterThanOrEquals('index', index, 0);
        Check.typeOf.number.lessThanOrEquals('index', index, 1);

        Check.typeOf.object('result', result);
        
        var startIndex = index * 2;
        var x = matrix[startIndex];
        var y = matrix[startIndex + 1];

        result.x = x;
        result.y = y;
        return result;
    };

    /**
     * Computes a new matrix that replaces the specified column in the provided matrix with the provided Cartesian2 instance.
     *
     * @param {Matrix2} matrix The matrix to use.
     * @param {Number} index The zero-based index of the column to set.
     * @param {Cartesian2} cartesian The Cartesian whose values will be assigned to the specified column.
     * @param {Cartesian2} result The object onto which to store the result.
     * @returns {Matrix2} The modified result parameter.
     *
     * @exception {DeveloperError} index must be 0 or 1.
     */
    Matrix2.setColumn = function(matrix, index, cartesian, result) {
                Check.typeOf.object('matrix', matrix);

        Check.typeOf.number.greaterThanOrEquals('index', index, 0);
        Check.typeOf.number.lessThanOrEquals('index', index, 1);

        Check.typeOf.object('cartesian', cartesian);
        Check.typeOf.object('result', result);
        
        result = Matrix2.clone(matrix, result);
        var startIndex = index * 2;
        result[startIndex] = cartesian.x;
        result[startIndex + 1] = cartesian.y;
        return result;
    };

    /**
     * Retrieves a copy of the matrix row at the provided index as a Cartesian2 instance.
     *
     * @param {Matrix2} matrix The matrix to use.
     * @param {Number} index The zero-based index of the row to retrieve.
     * @param {Cartesian2} result The object onto which to store the result.
     * @returns {Cartesian2} The modified result parameter.
     *
     * @exception {DeveloperError} index must be 0 or 1.
     */
    Matrix2.getRow = function(matrix, index, result) {
                Check.typeOf.object('matrix', matrix);

        Check.typeOf.number.greaterThanOrEquals('index', index, 0);
        Check.typeOf.number.lessThanOrEquals('index', index, 1);

        Check.typeOf.object('result', result);
        
        var x = matrix[index];
        var y = matrix[index + 2];

        result.x = x;
        result.y = y;
        return result;
    };

    /**
     * Computes a new matrix that replaces the specified row in the provided matrix with the provided Cartesian2 instance.
     *
     * @param {Matrix2} matrix The matrix to use.
     * @param {Number} index The zero-based index of the row to set.
     * @param {Cartesian2} cartesian The Cartesian whose values will be assigned to the specified row.
     * @param {Matrix2} result The object onto which to store the result.
     * @returns {Matrix2} The modified result parameter.
     *
     * @exception {DeveloperError} index must be 0 or 1.
     */
    Matrix2.setRow = function(matrix, index, cartesian, result) {
                Check.typeOf.object('matrix', matrix);

        Check.typeOf.number.greaterThanOrEquals('index', index, 0);
        Check.typeOf.number.lessThanOrEquals('index', index, 1);

        Check.typeOf.object('cartesian', cartesian);
        Check.typeOf.object('result', result);
        
        result = Matrix2.clone(matrix, result);
        result[index] = cartesian.x;
        result[index + 2] = cartesian.y;
        return result;
    };

    var scratchColumn = new Cartesian2();

    /**
     * Extracts the non-uniform scale assuming the matrix is an affine transformation.
     *
     * @param {Matrix2} matrix The matrix.
     * @param {Cartesian2} result The object onto which to store the result.
     * @returns {Cartesian2} The modified result parameter.
     */
    Matrix2.getScale = function(matrix, result) {
                Check.typeOf.object('matrix', matrix);
        Check.typeOf.object('result', result);
        
        result.x = Cartesian2.magnitude(Cartesian2.fromElements(matrix[0], matrix[1], scratchColumn));
        result.y = Cartesian2.magnitude(Cartesian2.fromElements(matrix[2], matrix[3], scratchColumn));
        return result;
    };

    var scratchScale = new Cartesian2();

    /**
     * Computes the maximum scale assuming the matrix is an affine transformation.
     * The maximum scale is the maximum length of the column vectors.
     *
     * @param {Matrix2} matrix The matrix.
     * @returns {Number} The maximum scale.
     */
    Matrix2.getMaximumScale = function(matrix) {
        Matrix2.getScale(matrix, scratchScale);
        return Cartesian2.maximumComponent(scratchScale);
    };

    /**
     * Computes the product of two matrices.
     *
     * @param {Matrix2} left The first matrix.
     * @param {Matrix2} right The second matrix.
     * @param {Matrix2} result The object onto which to store the result.
     * @returns {Matrix2} The modified result parameter.
     */
    Matrix2.multiply = function(left, right, result) {
                Check.typeOf.object('left', left);
        Check.typeOf.object('right', right);
        Check.typeOf.object('result', result);
        
        var column0Row0 = left[0] * right[0] + left[2] * right[1];
        var column1Row0 = left[0] * right[2] + left[2] * right[3];
        var column0Row1 = left[1] * right[0] + left[3] * right[1];
        var column1Row1 = left[1] * right[2] + left[3] * right[3];

        result[0] = column0Row0;
        result[1] = column0Row1;
        result[2] = column1Row0;
        result[3] = column1Row1;
        return result;
    };

    /**
     * Computes the sum of two matrices.
     *
     * @param {Matrix2} left The first matrix.
     * @param {Matrix2} right The second matrix.
     * @param {Matrix2} result The object onto which to store the result.
     * @returns {Matrix2} The modified result parameter.
     */
    Matrix2.add = function(left, right, result) {
                Check.typeOf.object('left', left);
        Check.typeOf.object('right', right);
        Check.typeOf.object('result', result);
        
        result[0] = left[0] + right[0];
        result[1] = left[1] + right[1];
        result[2] = left[2] + right[2];
        result[3] = left[3] + right[3];
        return result;
    };

    /**
     * Computes the difference of two matrices.
     *
     * @param {Matrix2} left The first matrix.
     * @param {Matrix2} right The second matrix.
     * @param {Matrix2} result The object onto which to store the result.
     * @returns {Matrix2} The modified result parameter.
     */
    Matrix2.subtract = function(left, right, result) {
                Check.typeOf.object('left', left);
        Check.typeOf.object('right', right);
        Check.typeOf.object('result', result);
        
        result[0] = left[0] - right[0];
        result[1] = left[1] - right[1];
        result[2] = left[2] - right[2];
        result[3] = left[3] - right[3];
        return result;
    };

    /**
     * Computes the product of a matrix and a column vector.
     *
     * @param {Matrix2} matrix The matrix.
     * @param {Cartesian2} cartesian The column.
     * @param {Cartesian2} result The object onto which to store the result.
     * @returns {Cartesian2} The modified result parameter.
     */
    Matrix2.multiplyByVector = function(matrix, cartesian, result) {
                Check.typeOf.object('matrix', matrix);
        Check.typeOf.object('cartesian', cartesian);
        Check.typeOf.object('result', result);
        
        var x = matrix[0] * cartesian.x + matrix[2] * cartesian.y;
        var y = matrix[1] * cartesian.x + matrix[3] * cartesian.y;

        result.x = x;
        result.y = y;
        return result;
    };

    /**
     * Computes the product of a matrix and a scalar.
     *
     * @param {Matrix2} matrix The matrix.
     * @param {Number} scalar The number to multiply by.
     * @param {Matrix2} result The object onto which to store the result.
     * @returns {Matrix2} The modified result parameter.
     */
    Matrix2.multiplyByScalar = function(matrix, scalar, result) {
                Check.typeOf.object('matrix', matrix);
        Check.typeOf.number('scalar', scalar);
        Check.typeOf.object('result', result);
        
        result[0] = matrix[0] * scalar;
        result[1] = matrix[1] * scalar;
        result[2] = matrix[2] * scalar;
        result[3] = matrix[3] * scalar;
        return result;
    };

    /**
     * Computes the product of a matrix times a (non-uniform) scale, as if the scale were a scale matrix.
     *
     * @param {Matrix2} matrix The matrix on the left-hand side.
     * @param {Cartesian2} scale The non-uniform scale on the right-hand side.
     * @param {Matrix2} result The object onto which to store the result.
     * @returns {Matrix2} The modified result parameter.
     *
     *
     * @example
     * // Instead of Cesium.Matrix2.multiply(m, Cesium.Matrix2.fromScale(scale), m);
     * Cesium.Matrix2.multiplyByScale(m, scale, m);
     *
     * @see Matrix2.fromScale
     * @see Matrix2.multiplyByUniformScale
     */
    Matrix2.multiplyByScale = function(matrix, scale, result) {
                Check.typeOf.object('matrix', matrix);
        Check.typeOf.object('scale', scale);
        Check.typeOf.object('result', result);
        
        result[0] = matrix[0] * scale.x;
        result[1] = matrix[1] * scale.x;
        result[2] = matrix[2] * scale.y;
        result[3] = matrix[3] * scale.y;
        return result;
    };

    /**
     * Creates a negated copy of the provided matrix.
     *
     * @param {Matrix2} matrix The matrix to negate.
     * @param {Matrix2} result The object onto which to store the result.
     * @returns {Matrix2} The modified result parameter.
     */
    Matrix2.negate = function(matrix, result) {
                Check.typeOf.object('matrix', matrix);
        Check.typeOf.object('result', result);
        
        result[0] = -matrix[0];
        result[1] = -matrix[1];
        result[2] = -matrix[2];
        result[3] = -matrix[3];
        return result;
    };

    /**
     * Computes the transpose of the provided matrix.
     *
     * @param {Matrix2} matrix The matrix to transpose.
     * @param {Matrix2} result The object onto which to store the result.
     * @returns {Matrix2} The modified result parameter.
     */
    Matrix2.transpose = function(matrix, result) {
                Check.typeOf.object('matrix', matrix);
        Check.typeOf.object('result', result);
        
        var column0Row0 = matrix[0];
        var column0Row1 = matrix[2];
        var column1Row0 = matrix[1];
        var column1Row1 = matrix[3];

        result[0] = column0Row0;
        result[1] = column0Row1;
        result[2] = column1Row0;
        result[3] = column1Row1;
        return result;
    };

    /**
     * Computes a matrix, which contains the absolute (unsigned) values of the provided matrix's elements.
     *
     * @param {Matrix2} matrix The matrix with signed elements.
     * @param {Matrix2} result The object onto which to store the result.
     * @returns {Matrix2} The modified result parameter.
     */
    Matrix2.abs = function(matrix, result) {
                Check.typeOf.object('matrix', matrix);
        Check.typeOf.object('result', result);
        
        result[0] = Math.abs(matrix[0]);
        result[1] = Math.abs(matrix[1]);
        result[2] = Math.abs(matrix[2]);
        result[3] = Math.abs(matrix[3]);

        return result;
    };

    /**
     * Compares the provided matrices componentwise and returns
     * <code>true</code> if they are equal, <code>false</code> otherwise.
     *
     * @param {Matrix2} [left] The first matrix.
     * @param {Matrix2} [right] The second matrix.
     * @returns {Boolean} <code>true</code> if left and right are equal, <code>false</code> otherwise.
     */
    Matrix2.equals = function(left, right) {
        return (left === right) ||
               (defined(left) &&
                defined(right) &&
                left[0] === right[0] &&
                left[1] === right[1] &&
                left[2] === right[2] &&
                left[3] === right[3]);
    };

    /**
     * @private
     */
    Matrix2.equalsArray = function(matrix, array, offset) {
        return matrix[0] === array[offset] &&
               matrix[1] === array[offset + 1] &&
               matrix[2] === array[offset + 2] &&
               matrix[3] === array[offset + 3];
    };

    /**
     * Compares the provided matrices componentwise and returns
     * <code>true</code> if they are within the provided epsilon,
     * <code>false</code> otherwise.
     *
     * @param {Matrix2} [left] The first matrix.
     * @param {Matrix2} [right] The second matrix.
     * @param {Number} epsilon The epsilon to use for equality testing.
     * @returns {Boolean} <code>true</code> if left and right are within the provided epsilon, <code>false</code> otherwise.
     */
    Matrix2.equalsEpsilon = function(left, right, epsilon) {
                Check.typeOf.number('epsilon', epsilon);
        
        return (left === right) ||
                (defined(left) &&
                defined(right) &&
                Math.abs(left[0] - right[0]) <= epsilon &&
                Math.abs(left[1] - right[1]) <= epsilon &&
                Math.abs(left[2] - right[2]) <= epsilon &&
                Math.abs(left[3] - right[3]) <= epsilon);
    };

    /**
     * An immutable Matrix2 instance initialized to the identity matrix.
     *
     * @type {Matrix2}
     * @constant
     */
    Matrix2.IDENTITY = freezeObject(new Matrix2(1.0, 0.0,
                                                0.0, 1.0));

    /**
     * An immutable Matrix2 instance initialized to the zero matrix.
     *
     * @type {Matrix2}
     * @constant
     */
    Matrix2.ZERO = freezeObject(new Matrix2(0.0, 0.0,
                                            0.0, 0.0));

    /**
     * The index into Matrix2 for column 0, row 0.
     *
     * @type {Number}
     * @constant
     *
     * @example
     * var matrix = new Cesium.Matrix2();
     * matrix[Cesium.Matrix2.COLUMN0ROW0] = 5.0; // set column 0, row 0 to 5.0
     */
    Matrix2.COLUMN0ROW0 = 0;

    /**
     * The index into Matrix2 for column 0, row 1.
     *
     * @type {Number}
     * @constant
     *
     * @example
     * var matrix = new Cesium.Matrix2();
     * matrix[Cesium.Matrix2.COLUMN0ROW1] = 5.0; // set column 0, row 1 to 5.0
     */
    Matrix2.COLUMN0ROW1 = 1;

    /**
     * The index into Matrix2 for column 1, row 0.
     *
     * @type {Number}
     * @constant
     *
     * @example
     * var matrix = new Cesium.Matrix2();
     * matrix[Cesium.Matrix2.COLUMN1ROW0] = 5.0; // set column 1, row 0 to 5.0
     */
    Matrix2.COLUMN1ROW0 = 2;

    /**
     * The index into Matrix2 for column 1, row 1.
     *
     * @type {Number}
     * @constant
     *
     * @example
     * var matrix = new Cesium.Matrix2();
     * matrix[Cesium.Matrix2.COLUMN1ROW1] = 5.0; // set column 1, row 1 to 5.0
     */
    Matrix2.COLUMN1ROW1 = 3;

    defineProperties(Matrix2.prototype, {
        /**
         * Gets the number of items in the collection.
         * @memberof Matrix2.prototype
         *
         * @type {Number}
         */
        length : {
            get : function() {
                return Matrix2.packedLength;
            }
        }
    });

    /**
     * Duplicates the provided Matrix2 instance.
     *
     * @param {Matrix2} [result] The object onto which to store the result.
     * @returns {Matrix2} The modified result parameter or a new Matrix2 instance if one was not provided.
     */
    Matrix2.prototype.clone = function(result) {
        return Matrix2.clone(this, result);
    };

    /**
     * Compares this matrix to the provided matrix componentwise and returns
     * <code>true</code> if they are equal, <code>false</code> otherwise.
     *
     * @param {Matrix2} [right] The right hand side matrix.
     * @returns {Boolean} <code>true</code> if they are equal, <code>false</code> otherwise.
     */
    Matrix2.prototype.equals = function(right) {
        return Matrix2.equals(this, right);
    };

    /**
     * Compares this matrix to the provided matrix componentwise and returns
     * <code>true</code> if they are within the provided epsilon,
     * <code>false</code> otherwise.
     *
     * @param {Matrix2} [right] The right hand side matrix.
     * @param {Number} epsilon The epsilon to use for equality testing.
     * @returns {Boolean} <code>true</code> if they are within the provided epsilon, <code>false</code> otherwise.
     */
    Matrix2.prototype.equalsEpsilon = function(right, epsilon) {
        return Matrix2.equalsEpsilon(this, right, epsilon);
    };

    /**
     * Creates a string representing this Matrix with each row being
     * on a separate line and in the format '(column0, column1)'.
     *
     * @returns {String} A string representing the provided Matrix with each row being on a separate line and in the format '(column0, column1)'.
     */
    Matrix2.prototype.toString = function() {
        return '(' + this[0] + ', ' + this[2] + ')\n' +
               '(' + this[1] + ', ' + this[3] + ')';
    };

    return Matrix2;
});

define('Core/PrimitiveType',[
        './freezeObject',
        './WebGLConstants'
    ], function(
        freezeObject,
        WebGLConstants) {
    'use strict';

    /**
     * The type of a geometric primitive, i.e., points, lines, and triangles.
     *
     * @exports PrimitiveType
     */
    var PrimitiveType = {
        /**
         * Points primitive where each vertex (or index) is a separate point.
         *
         * @type {Number}
         * @constant
         */
        POINTS : WebGLConstants.POINTS,

        /**
         * Lines primitive where each two vertices (or indices) is a line segment.  Line segments are not necessarily connected.
         *
         * @type {Number}
         * @constant
         */
        LINES : WebGLConstants.LINES,

        /**
         * Line loop primitive where each vertex (or index) after the first connects a line to
         * the previous vertex, and the last vertex implicitly connects to the first.
         *
         * @type {Number}
         * @constant
         */
        LINE_LOOP : WebGLConstants.LINE_LOOP,

        /**
         * Line strip primitive where each vertex (or index) after the first connects a line to the previous vertex.
         *
         * @type {Number}
         * @constant
         */
        LINE_STRIP : WebGLConstants.LINE_STRIP,

        /**
         * Triangles primitive where each three vertices (or indices) is a triangle.  Triangles do not necessarily share edges.
         *
         * @type {Number}
         * @constant
         */
        TRIANGLES : WebGLConstants.TRIANGLES,

        /**
         * Triangle strip primitive where each vertex (or index) after the first two connect to
         * the previous two vertices forming a triangle.  For example, this can be used to model a wall.
         *
         * @type {Number}
         * @constant
         */
        TRIANGLE_STRIP : WebGLConstants.TRIANGLE_STRIP,

        /**
         * Triangle fan primitive where each vertex (or index) after the first two connect to
         * the previous vertex and the first vertex forming a triangle.  For example, this can be used
         * to model a cone or circle.
         *
         * @type {Number}
         * @constant
         */
        TRIANGLE_FAN : WebGLConstants.TRIANGLE_FAN,

        /**
         * @private
         */
        validate : function(primitiveType) {
            return primitiveType === PrimitiveType.POINTS ||
                   primitiveType === PrimitiveType.LINES ||
                   primitiveType === PrimitiveType.LINE_LOOP ||
                   primitiveType === PrimitiveType.LINE_STRIP ||
                   primitiveType === PrimitiveType.TRIANGLES ||
                   primitiveType === PrimitiveType.TRIANGLE_STRIP ||
                   primitiveType === PrimitiveType.TRIANGLE_FAN;
        }
    };

    return freezeObject(PrimitiveType);
});

define('Core/binarySearch',[
        './Check'
    ], function(
        Check) {
    'use strict';

    /**
     * Finds an item in a sorted array.
     *
     * @exports binarySearch
     * @param {Array} array The sorted array to search.
     * @param {*} itemToFind The item to find in the array.
     * @param {binarySearch~Comparator} comparator The function to use to compare the item to
     *        elements in the array.
     * @returns {Number} The index of <code>itemToFind</code> in the array, if it exists.  If <code>itemToFind</code>
     *        does not exist, the return value is a negative number which is the bitwise complement (~)
     *        of the index before which the itemToFind should be inserted in order to maintain the
     *        sorted order of the array.
     *
     * @example
     * // Create a comparator function to search through an array of numbers.
     * function comparator(a, b) {
     *     return a - b;
     * };
     * var numbers = [0, 2, 4, 6, 8];
     * var index = Cesium.binarySearch(numbers, 6, comparator); // 3
     */
    function binarySearch(array, itemToFind, comparator) {
                Check.defined('array', array);
        Check.defined('itemToFind', itemToFind);
        Check.defined('comparator', comparator);
        
        var low = 0;
        var high = array.length - 1;
        var i;
        var comparison;

        while (low <= high) {
            i = ~~((low + high) / 2);
            comparison = comparator(array[i], itemToFind);
            if (comparison < 0) {
                low = i + 1;
                continue;
            }
            if (comparison > 0) {
                high = i - 1;
                continue;
            }
            return i;
        }
        return ~(high + 1);
    }

    /**
     * A function used to compare two items while performing a binary search.
     * @callback binarySearch~Comparator
     *
     * @param {*} a An item in the array.
     * @param {*} b The item being searched for.
     * @returns {Number} Returns a negative value if <code>a</code> is less than <code>b</code>,
     *          a positive value if <code>a</code> is greater than <code>b</code>, or
     *          0 if <code>a</code> is equal to <code>b</code>.
     *
     * @example
     * function compareNumbers(a, b) {
     *     return a - b;
     * }
     */

    return binarySearch;
});

define('Core/EarthOrientationParametersSample',[],function() {
    'use strict';

    /**
     * A set of Earth Orientation Parameters (EOP) sampled at a time.
     *
     * @alias EarthOrientationParametersSample
     * @constructor
     *
     * @param {Number} xPoleWander The pole wander about the X axis, in radians.
     * @param {Number} yPoleWander The pole wander about the Y axis, in radians.
     * @param {Number} xPoleOffset The offset to the Celestial Intermediate Pole (CIP) about the X axis, in radians.
     * @param {Number} yPoleOffset The offset to the Celestial Intermediate Pole (CIP) about the Y axis, in radians.
     * @param {Number} ut1MinusUtc The difference in time standards, UT1 - UTC, in seconds.
     *
     * @private
     */
    function EarthOrientationParametersSample(xPoleWander, yPoleWander, xPoleOffset, yPoleOffset, ut1MinusUtc) {
        /**
         * The pole wander about the X axis, in radians.
         * @type {Number}
         */
        this.xPoleWander = xPoleWander;

        /**
         * The pole wander about the Y axis, in radians.
         * @type {Number}
         */
        this.yPoleWander = yPoleWander;

        /**
         * The offset to the Celestial Intermediate Pole (CIP) about the X axis, in radians.
         * @type {Number}
         */
        this.xPoleOffset = xPoleOffset;

        /**
         * The offset to the Celestial Intermediate Pole (CIP) about the Y axis, in radians.
         * @type {Number}
         */
        this.yPoleOffset = yPoleOffset;

        /**
         * The difference in time standards, UT1 - UTC, in seconds.
         * @type {Number}
         */
        this.ut1MinusUtc = ut1MinusUtc;
    }

    return EarthOrientationParametersSample;
});

/**
@license
sprintf.js from the php.js project - https://github.com/kvz/phpjs
Directly from https://github.com/kvz/phpjs/blob/master/functions/strings/sprintf.js

php.js is copyright 2012 Kevin van Zonneveld.

Portions copyright Brett Zamir (http://brett-zamir.me), Kevin van Zonneveld
(http://kevin.vanzonneveld.net), Onno Marsman, Theriault, Michael White
(http://getsprink.com), Waldo Malqui Silva, Paulo Freitas, Jack, Jonas
Raoni Soares Silva (http://www.jsfromhell.com), Philip Peterson, Legaev
Andrey, Ates Goral (http://magnetiq.com), Alex, Ratheous, Martijn Wieringa,
Rafa? Kukawski (http://blog.kukawski.pl), lmeyrick
(https://sourceforge.net/projects/bcmath-js/), Nate, Philippe Baumann,
Enrique Gonzalez, Webtoolkit.info (http://www.webtoolkit.info/), Carlos R.
L. Rodrigues (http://www.jsfromhell.com), Ash Searle
(http://hexmen.com/blog/), Jani Hartikainen, travc, Ole Vrijenhoek,
Erkekjetter, Michael Grier, Rafa? Kukawski (http://kukawski.pl), Johnny
Mast (http://www.phpvrouwen.nl), T.Wild, d3x,
http://stackoverflow.com/questions/57803/how-to-convert-decimal-to-hex-in-javascript,
Rafa? Kukawski (http://blog.kukawski.pl/), stag019, pilus, WebDevHobo
(http://webdevhobo.blogspot.com/), marrtins, GeekFG
(http://geekfg.blogspot.com), Andrea Giammarchi
(http://webreflection.blogspot.com), Arpad Ray (mailto:arpad@php.net),
gorthaur, Paul Smith, Tim de Koning (http://www.kingsquare.nl), Joris, Oleg
Eremeev, Steve Hilder, majak, gettimeofday, KELAN, Josh Fraser
(http://onlineaspect.com/2007/06/08/auto-detect-a-time-zone-with-javascript/),
Marc Palau, Martin
(http://www.erlenwiese.de/), Breaking Par Consulting Inc
(http://www.breakingpar.com/bkp/home.nsf/0/87256B280015193F87256CFB006C45F7),
Chris, Mirek Slugen, saulius, Alfonso Jimenez
(http://www.alfonsojimenez.com), Diplom@t (http://difane.com/), felix,
Mailfaker (http://www.weedem.fr/), Tyler Akins (http://rumkin.com), Caio
Ariede (http://caioariede.com), Robin, Kankrelune
(http://www.webfaktory.info/), Karol Kowalski, Imgen Tata
(http://www.myipdf.com/), mdsjack (http://www.mdsjack.bo.it), Dreamer,
Felix Geisendoerfer (http://www.debuggable.com/felix), Lars Fischer, AJ,
David, Aman Gupta, Michael White, Public Domain
(http://www.json.org/json2.js), Steven Levithan
(http://blog.stevenlevithan.com), Sakimori, Pellentesque Malesuada,
Thunder.m, Dj (http://phpjs.org/functions/htmlentities:425#comment_134018),
Steve Clay, David James, Francois, class_exists, nobbler, T. Wild, Itsacon
(http://www.itsacon.net/), date, Ole Vrijenhoek (http://www.nervous.nl/),
Fox, Raphael (Ao RUDLER), Marco, noname, Mateusz "loonquawl" Zalega, Frank
Forte, Arno, ger, mktime, john (http://www.jd-tech.net), Nick Kolosov
(http://sammy.ru), marc andreu, Scott Cariss, Douglas Crockford
(http://javascript.crockford.com), madipta, Slawomir Kaniecki,
ReverseSyntax, Nathan, Alex Wilson, kenneth, Bayron Guevara, Adam Wallner
(http://web2.bitbaro.hu/), paulo kuong, jmweb, Lincoln Ramsay, djmix,
Pyerre, Jon Hohle, Thiago Mata (http://thiagomata.blog.com), lmeyrick
(https://sourceforge.net/projects/bcmath-js/this.), Linuxworld, duncan,
Gilbert, Sanjoy Roy, Shingo, sankai, Oskar Larsson H?gfeldt
(http://oskar-lh.name/), Denny Wardhana, 0m3r, Everlasto, Subhasis Deb,
josh, jd, Pier Paolo Ramon (http://www.mastersoup.com/), P, merabi, Soren
Hansen, Eugene Bulkin (http://doubleaw.com/), Der Simon
(http://innerdom.sourceforge.net/), echo is bad, Ozh, XoraX
(http://www.xorax.info), EdorFaus, JB, J A R, Marc Jansen, Francesco, LH,
Stoyan Kyosev (http://www.svest.org/), nord_ua, omid
(http://phpjs.org/functions/380:380#comment_137122), Brad Touesnard, MeEtc
(http://yass.meetcweb.com), Peter-Paul Koch
(http://www.quirksmode.org/js/beat.html), Olivier Louvignes
(http://mg-crea.com/), T0bsn, Tim Wiel, Bryan Elliott, Jalal Berrami,
Martin, JT, David Randall, Thomas Beaucourt (http://www.webapp.fr), taith,
vlado houba, Pierre-Luc Paour, Kristof Coomans (SCK-CEN Belgian Nucleair
Research Centre), Martin Pool, Kirk Strobeck, Rick Waldron, Brant Messenger
(http://www.brantmessenger.com/), Devan Penner-Woelk, Saulo Vallory, Wagner
B. Soares, Artur Tchernychev, Valentina De Rosa, Jason Wong
(http://carrot.org/), Christoph, Daniel Esteban, strftime, Mick@el, rezna,
Simon Willison (http://simonwillison.net), Anton Ongson, Gabriel Paderni,
Marco van Oort, penutbutterjelly, Philipp Lenssen, Bjorn Roesbeke
(http://www.bjornroesbeke.be/), Bug?, Eric Nagel, Tomasz Wesolowski,
Evertjan Garretsen, Bobby Drake, Blues (http://tech.bluesmoon.info/), Luke
Godfrey, Pul, uestla, Alan C, Ulrich, Rafal Kukawski, Yves Sucaet,
sowberry, Norman "zEh" Fuchs, hitwork, Zahlii, johnrembo, Nick Callen,
Steven Levithan (stevenlevithan.com), ejsanders, Scott Baker, Brian Tafoya
(http://www.premasolutions.com/), Philippe Jausions
(http://pear.php.net/user/jausions), Aidan Lister
(http://aidanlister.com/), Rob, e-mike, HKM, ChaosNo1, metjay, strcasecmp,
strcmp, Taras Bogach, jpfle, Alexander Ermolaev
(http://snippets.dzone.com/user/AlexanderErmolaev), DxGx, kilops, Orlando,
dptr1988, Le Torbi, James (http://www.james-bell.co.uk/), Pedro Tainha
(http://www.pedrotainha.com), James, Arnout Kazemier
(http://www.3rd-Eden.com), Chris McMacken, gabriel paderni, Yannoo,
FGFEmperor, baris ozdil, Tod Gentille, Greg Frazier, jakes, 3D-GRAF, Allan
Jensen (http://www.winternet.no), Howard Yeend, Benjamin Lupton, davook,
daniel airton wermann (http://wermann.com.br), Atli T¨®r, Maximusya, Ryan
W Tenney (http://ryan.10e.us), Alexander M Beedie, fearphage
(http://http/my.opera.com/fearphage/), Nathan Sepulveda, Victor, Matteo,
Billy, stensi, Cord, Manish, T.J. Leahy, Riddler
(http://www.frontierwebdev.com/), Rafa? Kukawski, FremyCompany, Matt
Bradley, Tim de Koning, Luis Salazar (http://www.freaky-media.com/), Diogo
Resende, Rival, Andrej Pavlovic, Garagoth, Le Torbi
(http://www.letorbi.de/), Dino, Josep Sanz (http://www.ws3.es/), rem,
Russell Walker (http://www.nbill.co.uk/), Jamie Beck
(http://www.terabit.ca/), setcookie, Michael, YUI Library:
http://developer.yahoo.com/yui/docs/YAHOO.util.DateLocale.html, Blues at
http://hacks.bluesmoon.info/strftime/strftime.js, Ben
(http://benblume.co.uk/), DtTvB
(http://dt.in.th/2008-09-16.string-length-in-bytes.html), Andreas, William,
meo, incidence, Cagri Ekin, Amirouche, Amir Habibi
(http://www.residence-mixte.com/), Luke Smith (http://lucassmith.name),
Kheang Hok Chin (http://www.distantia.ca/), Jay Klehr, Lorenzo Pisani,
Tony, Yen-Wei Liu, Greenseed, mk.keck, Leslie Hoare, dude, booeyOH, Ben
Bryan

Licensed under the MIT (MIT-LICENSE.txt) license.

Permission is hereby granted, free of charge, to any person obtaining a
copy of this software and associated documentation files (the
"Software"), to deal in the Software without restriction, including
without limitation the rights to use, copy, modify, merge, publish,
distribute, sublicense, and/or sell copies of the Software, and to
permit persons to whom the Software is furnished to do so, subject to
the following conditions:

The above copyright notice and this permission notice shall be included
in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
IN NO EVENT SHALL KEVIN VAN ZONNEVELD BE LIABLE FOR ANY CLAIM, DAMAGES
OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
OTHER DEALINGS IN THE SOFTWARE.
*/

define('ThirdParty/sprintf',[],function() {

function sprintf () {
  // http://kevin.vanzonneveld.net
  // +   original by: Ash Searle (http://hexmen.com/blog/)
  // + namespaced by: Michael White (http://getsprink.com)
  // +    tweaked by: Jack
  // +   improved by: Kevin van Zonneveld (http://kevin.vanzonneveld.net)
  // +      input by: Paulo Freitas
  // +   improved by: Kevin van Zonneveld (http://kevin.vanzonneveld.net)
  // +      input by: Brett Zamir (http://brett-zamir.me)
  // +   improved by: Kevin van Zonneveld (http://kevin.vanzonneveld.net)
  // +   improved by: Dj
  // +   improved by: Allidylls
  // *     example 1: sprintf("%01.2f", 123.1);
  // *     returns 1: 123.10
  // *     example 2: sprintf("[%10s]", 'monkey');
  // *     returns 2: '[    monkey]'
  // *     example 3: sprintf("[%'#10s]", 'monkey');
  // *     returns 3: '[####monkey]'
  // *     example 4: sprintf("%d", 123456789012345);
  // *     returns 4: '123456789012345'
  var regex = /%%|%(\d+\$)?([-+\'#0 ]*)(\*\d+\$|\*|\d+)?(\.(\*\d+\$|\*|\d+))?([scboxXuideEfFgG])/g;
  var a = arguments,
    i = 0,
    format = a[i++];

  // pad()
  var pad = function (str, len, chr, leftJustify) {
    if (!chr) {
      chr = ' ';
    }

    var padding = (str.length >= len) ? '' : Array(1 + len - str.length >>> 0).join(chr);
    return leftJustify ? str + padding : padding + str;
  };

  // justify()
  var justify = function (value, prefix, leftJustify, minWidth, zeroPad, customPadChar) {
    var diff = minWidth - value.length;
    if (diff > 0) {
      if (leftJustify || !zeroPad) {
        value = pad(value, minWidth, customPadChar, leftJustify);
      } else {
        value = value.slice(0, prefix.length) + pad('', diff, '0', true) + value.slice(prefix.length);
      }
    }
    return value;
  };

  // formatBaseX()
  var formatBaseX = function (value, base, prefix, leftJustify, minWidth, precision, zeroPad) {
    // Note: casts negative numbers to positive ones
    var number = value >>> 0;
    prefix = prefix && number && {
      '2': '0b',
      '8': '0',
      '16': '0x'
    }[base] || '';
    value = prefix + pad(number.toString(base), precision || 0, '0', false);
    return justify(value, prefix, leftJustify, minWidth, zeroPad);
  };

  // formatString()
  var formatString = function (value, leftJustify, minWidth, precision, zeroPad, customPadChar) {
    if (precision != null) {
      value = value.slice(0, precision);
    }
    return justify(value, '', leftJustify, minWidth, zeroPad, customPadChar);
  };

  // doFormat()
  var doFormat = function (substring, valueIndex, flags, minWidth, _, precision, type) {
    var number;
    var prefix;
    var method;
    var textTransform;
    var value;

    if (substring == '%%') {
      return '%';
    }

    // parse flags
    var leftJustify = false,
      positivePrefix = '',
      zeroPad = false,
      prefixBaseX = false,
      customPadChar = ' ';
    var flagsl = flags.length;
    for (var j = 0; flags && j < flagsl; j++) {
      switch (flags.charAt(j)) {
      case ' ':
        positivePrefix = ' ';
        break;
      case '+':
        positivePrefix = '+';
        break;
      case '-':
        leftJustify = true;
        break;
      case "'":
        customPadChar = flags.charAt(j + 1);
        break;
      case '0':
        zeroPad = true;
        break;
      case '#':
        prefixBaseX = true;
        break;
      }
    }

    // parameters may be null, undefined, empty-string or real valued
    // we want to ignore null, undefined and empty-string values
    if (!minWidth) {
      minWidth = 0;
    } else if (minWidth == '*') {
      minWidth = +a[i++];
    } else if (minWidth.charAt(0) == '*') {
      minWidth = +a[minWidth.slice(1, -1)];
    } else {
      minWidth = +minWidth;
    }

    // Note: undocumented perl feature:
    if (minWidth < 0) {
      minWidth = -minWidth;
      leftJustify = true;
    }

    if (!isFinite(minWidth)) {
      throw new Error('sprintf: (minimum-)width must be finite');
    }

    if (!precision) {
      precision = 'fFeE'.indexOf(type) > -1 ? 6 : (type == 'd') ? 0 : undefined;
    } else if (precision == '*') {
      precision = +a[i++];
    } else if (precision.charAt(0) == '*') {
      precision = +a[precision.slice(1, -1)];
    } else {
      precision = +precision;
    }

    // grab value using valueIndex if required?
    value = valueIndex ? a[valueIndex.slice(0, -1)] : a[i++];

    switch (type) {
    case 's':
      return formatString(String(value), leftJustify, minWidth, precision, zeroPad, customPadChar);
    case 'c':
      return formatString(String.fromCharCode(+value), leftJustify, minWidth, precision, zeroPad);
    case 'b':
      return formatBaseX(value, 2, prefixBaseX, leftJustify, minWidth, precision, zeroPad);
    case 'o':
      return formatBaseX(value, 8, prefixBaseX, leftJustify, minWidth, precision, zeroPad);
    case 'x':
      return formatBaseX(value, 16, prefixBaseX, leftJustify, minWidth, precision, zeroPad);
    case 'X':
      return formatBaseX(value, 16, prefixBaseX, leftJustify, minWidth, precision, zeroPad).toUpperCase();
    case 'u':
      return formatBaseX(value, 10, prefixBaseX, leftJustify, minWidth, precision, zeroPad);
    case 'i':
    case 'd':
      number = +value || 0;
      number = Math.round(number - number % 1); // Plain Math.round doesn't just truncate
      prefix = number < 0 ? '-' : positivePrefix;
      value = prefix + pad(String(Math.abs(number)), precision, '0', false);
      return justify(value, prefix, leftJustify, minWidth, zeroPad);
    case 'e':
    case 'E':
    case 'f': // Should handle locales (as per setlocale)
    case 'F':
    case 'g':
    case 'G':
      number = +value;
      prefix = number < 0 ? '-' : positivePrefix;
      method = ['toExponential', 'toFixed', 'toPrecision']['efg'.indexOf(type.toLowerCase())];
      textTransform = ['toString', 'toUpperCase']['eEfFgG'.indexOf(type) % 2];
      value = prefix + Math.abs(number)[method](precision);
      return justify(value, prefix, leftJustify, minWidth, zeroPad)[textTransform]();
    default:
      return substring;
    }
  };

  return format.replace(regex, doFormat);
}

return sprintf;
});

define('Core/GregorianDate',[],function() {
    'use strict';

    /**
     * Represents a Gregorian date in a more precise format than the JavaScript Date object.
     * In addition to submillisecond precision, this object can also represent leap seconds.
     * @alias GregorianDate
     * @constructor
     *
     * @see JulianDate#toGregorianDate
     */
    function GregorianDate(year, month, day, hour, minute, second, millisecond, isLeapSecond) {
        /**
         * Gets or sets the year as a whole number.
         * @type {Number}
         */
        this.year = year;
        /**
         * Gets or sets the month as a whole number with range [1, 12].
         * @type {Number}
         */
        this.month = month;
        /**
         * Gets or sets the day of the month as a whole number starting at 1.
         * @type {Number}
         */
        this.day = day;
        /**
         * Gets or sets the hour as a whole number with range [0, 23].
         * @type {Number}
         */
        this.hour = hour;
        /**
         * Gets or sets the minute of the hour as a whole number with range [0, 59].
         * @type {Number}
         */
        this.minute = minute;
        /**
         * Gets or sets the second of the minute as a whole number with range [0, 60], with 60 representing a leap second.
         * @type {Number}
         */
        this.second = second;
        /**
         * Gets or sets the millisecond of the second as a floating point number with range [0.0, 1000.0).
         * @type {Number}
         */
        this.millisecond = millisecond;
        /**
         * Gets or sets whether this time is during a leap second.
         * @type {Boolean}
         */
        this.isLeapSecond = isLeapSecond;
    }

    return GregorianDate;
});

define('Core/isLeapYear',[
        './DeveloperError'
    ], function(
        DeveloperError) {
    'use strict';

    /**
     * Determines if a given date is a leap year.
     *
     * @exports isLeapYear
     *
     * @param {Number} year The year to be tested.
     * @returns {Boolean} True if <code>year</code> is a leap year.
     *
     * @example
     * var leapYear = Cesium.isLeapYear(2000); // true
     */
    function isLeapYear(year) {
                if (year === null || isNaN(year)) {
            throw new DeveloperError('year is required and must be a number.');
        }
        
        return ((year % 4 === 0) && (year % 100 !== 0)) || (year % 400 === 0);
    }

    return isLeapYear;
});

define('Core/LeapSecond',[],function() {
    'use strict';

    /**
     * Describes a single leap second, which is constructed from a {@link JulianDate} and a
     * numerical offset representing the number of seconds TAI is ahead of the UTC time standard.
     * @alias LeapSecond
     * @constructor
     *
     * @param {JulianDate} [date] A Julian date representing the time of the leap second.
     * @param {Number} [offset] The cumulative number of seconds that TAI is ahead of UTC at the provided date.
     */
    function LeapSecond(date, offset) {
        /**
         * Gets or sets the date at which this leap second occurs.
         * @type {JulianDate}
         */
        this.julianDate = date;

        /**
         * Gets or sets the cumulative number of seconds between the UTC and TAI time standards at the time
         * of this leap second.
         * @type {Number}
         */
        this.offset = offset;
    }

    return LeapSecond;
});

define('Core/TimeConstants',[
        './freezeObject'
    ], function(
        freezeObject) {
    'use strict';

    /**
     * Constants for time conversions like those done by {@link JulianDate}.
     *
     * @exports TimeConstants
     *
     * @see JulianDate
     *
     * @private
     */
    var TimeConstants = {
        /**
         * The number of seconds in one millisecond: <code>0.001</code>
         * @type {Number}
         * @constant
         */
        SECONDS_PER_MILLISECOND : 0.001,

        /**
         * The number of seconds in one minute: <code>60</code>.
         * @type {Number}
         * @constant
         */
        SECONDS_PER_MINUTE : 60.0,

        /**
         * The number of minutes in one hour: <code>60</code>.
         * @type {Number}
         * @constant
         */
        MINUTES_PER_HOUR : 60.0,

        /**
         * The number of hours in one day: <code>24</code>.
         * @type {Number}
         * @constant
         */
        HOURS_PER_DAY : 24.0,

        /**
         * The number of seconds in one hour: <code>3600</code>.
         * @type {Number}
         * @constant
         */
        SECONDS_PER_HOUR : 3600.0,

        /**
         * The number of minutes in one day: <code>1440</code>.
         * @type {Number}
         * @constant
         */
        MINUTES_PER_DAY : 1440.0,

        /**
         * The number of seconds in one day, ignoring leap seconds: <code>86400</code>.
         * @type {Number}
         * @constant
         */
        SECONDS_PER_DAY : 86400.0,

        /**
         * The number of days in one Julian century: <code>36525</code>.
         * @type {Number}
         * @constant
         */
        DAYS_PER_JULIAN_CENTURY : 36525.0,

        /**
         * One trillionth of a second.
         * @type {Number}
         * @constant
         */
        PICOSECOND : 0.000000001,

        /**
         * The number of days to subtract from a Julian date to determine the
         * modified Julian date, which gives the number of days since midnight
         * on November 17, 1858.
         * @type {Number}
         * @constant
         */
        MODIFIED_JULIAN_DATE_DIFFERENCE : 2400000.5
    };

    return freezeObject(TimeConstants);
});

define('Core/TimeStandard',[
        './freezeObject'
    ], function(
        freezeObject) {
    'use strict';

    /**
     * Provides the type of time standards which JulianDate can take as input.
     *
     * @exports TimeStandard
     *
     * @see JulianDate
     */
    var TimeStandard = {
        /**
         * Represents the coordinated Universal Time (UTC) time standard.
         *
         * UTC is related to TAI according to the relationship
         * <code>UTC = TAI - deltaT</code> where <code>deltaT</code> is the number of leap
         * seconds which have been introduced as of the time in TAI.
         *
         * @type {Number}
         * @constant
         */
        UTC : 0,

        /**
         * Represents the International Atomic Time (TAI) time standard.
         * TAI is the principal time standard to which the other time standards are related.
         *
         * @type {Number}
         * @constant
         */
        TAI : 1
    };

    return freezeObject(TimeStandard);
});

define('Core/JulianDate',[
        '../ThirdParty/sprintf',
        './binarySearch',
        './defaultValue',
        './defined',
        './DeveloperError',
        './GregorianDate',
        './isLeapYear',
        './LeapSecond',
        './TimeConstants',
        './TimeStandard'
    ], function(
        sprintf,
        binarySearch,
        defaultValue,
        defined,
        DeveloperError,
        GregorianDate,
        isLeapYear,
        LeapSecond,
        TimeConstants,
        TimeStandard) {
    'use strict';

    var gregorianDateScratch = new GregorianDate();
    var daysInMonth = [31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31];
    var daysInLeapFeburary = 29;

    function compareLeapSecondDates(leapSecond, dateToFind) {
        return JulianDate.compare(leapSecond.julianDate, dateToFind.julianDate);
    }

    // we don't really need a leap second instance, anything with a julianDate property will do
    var binarySearchScratchLeapSecond = new LeapSecond();

    function convertUtcToTai(julianDate) {
        //Even though julianDate is in UTC, we'll treat it as TAI and
        //search the leap second table for it.
        binarySearchScratchLeapSecond.julianDate = julianDate;
        var leapSeconds = JulianDate.leapSeconds;
        var index = binarySearch(leapSeconds, binarySearchScratchLeapSecond, compareLeapSecondDates);

        if (index < 0) {
            index = ~index;
        }

        if (index >= leapSeconds.length) {
            index = leapSeconds.length - 1;
        }

        var offset = leapSeconds[index].offset;
        if (index > 0) {
            //Now we have the index of the closest leap second that comes on or after our UTC time.
            //However, if the difference between the UTC date being converted and the TAI
            //defined leap second is greater than the offset, we are off by one and need to use
            //the previous leap second.
            var difference = JulianDate.secondsDifference(leapSeconds[index].julianDate, julianDate);
            if (difference > offset) {
                index--;
                offset = leapSeconds[index].offset;
            }
        }

        JulianDate.addSeconds(julianDate, offset, julianDate);
    }

    function convertTaiToUtc(julianDate, result) {
        binarySearchScratchLeapSecond.julianDate = julianDate;
        var leapSeconds = JulianDate.leapSeconds;
        var index = binarySearch(leapSeconds, binarySearchScratchLeapSecond, compareLeapSecondDates);
        if (index < 0) {
            index = ~index;
        }

        //All times before our first leap second get the first offset.
        if (index === 0) {
            return JulianDate.addSeconds(julianDate, -leapSeconds[0].offset, result);
        }

        //All times after our leap second get the last offset.
        if (index >= leapSeconds.length) {
            return JulianDate.addSeconds(julianDate, -leapSeconds[index - 1].offset, result);
        }

        //Compute the difference between the found leap second and the time we are converting.
        var difference = JulianDate.secondsDifference(leapSeconds[index].julianDate, julianDate);

        if (difference === 0) {
            //The date is in our leap second table.
            return JulianDate.addSeconds(julianDate, -leapSeconds[index].offset, result);
        }

        if (difference <= 1.0) {
            //The requested date is during the moment of a leap second, then we cannot convert to UTC
            return undefined;
        }

        //The time is in between two leap seconds, index is the leap second after the date
        //we're converting, so we subtract one to get the correct LeapSecond instance.
        return JulianDate.addSeconds(julianDate, -leapSeconds[--index].offset, result);
    }

    function setComponents(wholeDays, secondsOfDay, julianDate) {
        var extraDays = (secondsOfDay / TimeConstants.SECONDS_PER_DAY) | 0;
        wholeDays += extraDays;
        secondsOfDay -= TimeConstants.SECONDS_PER_DAY * extraDays;

        if (secondsOfDay < 0) {
            wholeDays--;
            secondsOfDay += TimeConstants.SECONDS_PER_DAY;
        }

        julianDate.dayNumber = wholeDays;
        julianDate.secondsOfDay = secondsOfDay;
        return julianDate;
    }

    function computeJulianDateComponents(year, month, day, hour, minute, second, millisecond) {
        // Algorithm from page 604 of the Explanatory Supplement to the
        // Astronomical Almanac (Seidelmann 1992).

        var a = ((month - 14) / 12) | 0;
        var b = year + 4800 + a;
        var dayNumber = (((1461 * b) / 4) | 0) + (((367 * (month - 2 - 12 * a)) / 12) | 0) - (((3 * (((b + 100) / 100) | 0)) / 4) | 0) + day - 32075;

        // JulianDates are noon-based
        hour = hour - 12;
        if (hour < 0) {
            hour += 24;
        }

        var secondsOfDay = second + ((hour * TimeConstants.SECONDS_PER_HOUR) + (minute * TimeConstants.SECONDS_PER_MINUTE) + (millisecond * TimeConstants.SECONDS_PER_MILLISECOND));

        if (secondsOfDay >= 43200.0) {
            dayNumber -= 1;
        }

        return [dayNumber, secondsOfDay];
    }

    //Regular expressions used for ISO8601 date parsing.
    //YYYY
    var matchCalendarYear = /^(\d{4})$/;
    //YYYY-MM (YYYYMM is invalid)
    var matchCalendarMonth = /^(\d{4})-(\d{2})$/;
    //YYYY-DDD or YYYYDDD
    var matchOrdinalDate = /^(\d{4})-?(\d{3})$/;
    //YYYY-Www or YYYYWww or YYYY-Www-D or YYYYWwwD
    var matchWeekDate = /^(\d{4})-?W(\d{2})-?(\d{1})?$/;
    //YYYY-MM-DD or YYYYMMDD
    var matchCalendarDate = /^(\d{4})-?(\d{2})-?(\d{2})$/;
    // Match utc offset
    var utcOffset = /([Z+\-])?(\d{2})?:?(\d{2})?$/;
    // Match hours HH or HH.xxxxx
    var matchHours = /^(\d{2})(\.\d+)?/.source + utcOffset.source;
    // Match hours/minutes HH:MM HHMM.xxxxx
    var matchHoursMinutes = /^(\d{2}):?(\d{2})(\.\d+)?/.source + utcOffset.source;
    // Match hours/minutes HH:MM:SS HHMMSS.xxxxx
    var matchHoursMinutesSeconds = /^(\d{2}):?(\d{2}):?(\d{2})(\.\d+)?/.source + utcOffset.source;

    var iso8601ErrorMessage = 'Invalid ISO 8601 date.';

    /**
     * Represents an astronomical Julian date, which is the number of days since noon on January 1, -4712 (4713 BC).
     * For increased precision, this class stores the whole number part of the date and the seconds
     * part of the date in separate components.  In order to be safe for arithmetic and represent
     * leap seconds, the date is always stored in the International Atomic Time standard
     * {@link TimeStandard.TAI}.
     * @alias JulianDate
     * @constructor
     *
     * @param {Number} [julianDayNumber=0.0] The Julian Day Number representing the number of whole days.  Fractional days will also be handled correctly.
     * @param {Number} [secondsOfDay=0.0] The number of seconds into the current Julian Day Number.  Fractional seconds, negative seconds and seconds greater than a day will be handled correctly.
     * @param {TimeStandard} [timeStandard=TimeStandard.UTC] The time standard in which the first two parameters are defined.
     */
    function JulianDate(julianDayNumber, secondsOfDay, timeStandard) {
        /**
         * Gets or sets the number of whole days.
         * @type {Number}
         */
        this.dayNumber = undefined;

        /**
         * Gets or sets the number of seconds into the current day.
         * @type {Number}
         */
        this.secondsOfDay = undefined;

        julianDayNumber = defaultValue(julianDayNumber, 0.0);
        secondsOfDay = defaultValue(secondsOfDay, 0.0);
        timeStandard = defaultValue(timeStandard, TimeStandard.UTC);

        //If julianDayNumber is fractional, make it an integer and add the number of seconds the fraction represented.
        var wholeDays = julianDayNumber | 0;
        secondsOfDay = secondsOfDay + (julianDayNumber - wholeDays) * TimeConstants.SECONDS_PER_DAY;

        setComponents(wholeDays, secondsOfDay, this);

        if (timeStandard === TimeStandard.UTC) {
            convertUtcToTai(this);
        }
    }

    /**
     * Creates a new instance from a GregorianDate.
     *
     * @param {GregorianDate} date A GregorianDate.
     * @param {JulianDate} [result] An existing instance to use for the result.
     * @returns {JulianDate} The modified result parameter or a new instance if none was provided.
     *
     * @exception {DeveloperError} date must be a valid GregorianDate.
     */
    JulianDate.fromGregorianDate = function(date, result) {
                if (!(date instanceof GregorianDate)) {
            throw new DeveloperError('date must be a valid GregorianDate.');
        }
        
        var components = computeJulianDateComponents(date.year, date.month, date.day, date.hour, date.minute, date.second, date.millisecond);
        if (!defined(result)) {
            return new JulianDate(components[0], components[1], TimeStandard.UTC);
        }
        setComponents(components[0], components[1], result);
        convertUtcToTai(result);
        return result;
    };

    /**
     * Creates a new instance from a JavaScript Date.
     *
     * @param {Date} date A JavaScript Date.
     * @param {JulianDate} [result] An existing instance to use for the result.
     * @returns {JulianDate} The modified result parameter or a new instance if none was provided.
     *
     * @exception {DeveloperError} date must be a valid JavaScript Date.
     */
    JulianDate.fromDate = function(date, result) {
                if (!(date instanceof Date) || isNaN(date.getTime())) {
            throw new DeveloperError('date must be a valid JavaScript Date.');
        }
        
        var components = computeJulianDateComponents(date.getUTCFullYear(), date.getUTCMonth() + 1, date.getUTCDate(), date.getUTCHours(), date.getUTCMinutes(), date.getUTCSeconds(), date.getUTCMilliseconds());
        if (!defined(result)) {
            return new JulianDate(components[0], components[1], TimeStandard.UTC);
        }
        setComponents(components[0], components[1], result);
        convertUtcToTai(result);
        return result;
    };

    /**
     * Creates a new instance from a from an {@link http://en.wikipedia.org/wiki/ISO_8601|ISO 8601} date.
     * This method is superior to <code>Date.parse</code> because it will handle all valid formats defined by the ISO 8601
     * specification, including leap seconds and sub-millisecond times, which discarded by most JavaScript implementations.
     *
     * @param {String} iso8601String An ISO 8601 date.
     * @param {JulianDate} [result] An existing instance to use for the result.
     * @returns {JulianDate} The modified result parameter or a new instance if none was provided.
     *
     * @exception {DeveloperError} Invalid ISO 8601 date.
     */
    JulianDate.fromIso8601 = function(iso8601String, result) {
                if (typeof iso8601String !== 'string') {
            throw new DeveloperError(iso8601ErrorMessage);
        }
        
        //Comma and decimal point both indicate a fractional number according to ISO 8601,
        //start out by blanket replacing , with . which is the only valid such symbol in JS.
        iso8601String = iso8601String.replace(',', '.');

        //Split the string into its date and time components, denoted by a mandatory T
        var tokens = iso8601String.split('T');
        var year;
        var month = 1;
        var day = 1;
        var hour = 0;
        var minute = 0;
        var second = 0;
        var millisecond = 0;

        //Lacking a time is okay, but a missing date is illegal.
        var date = tokens[0];
        var time = tokens[1];
        var tmp;
        var inLeapYear;
                if (!defined(date)) {
            throw new DeveloperError(iso8601ErrorMessage);
        }

        var dashCount;
        
        //First match the date against possible regular expressions.
        tokens = date.match(matchCalendarDate);
        if (tokens !== null) {
                        dashCount = date.split('-').length - 1;
            if (dashCount > 0 && dashCount !== 2) {
                throw new DeveloperError(iso8601ErrorMessage);
            }
                        year = +tokens[1];
            month = +tokens[2];
            day = +tokens[3];
        } else {
            tokens = date.match(matchCalendarMonth);
            if (tokens !== null) {
                year = +tokens[1];
                month = +tokens[2];
            } else {
                tokens = date.match(matchCalendarYear);
                if (tokens !== null) {
                    year = +tokens[1];
                } else {
                    //Not a year/month/day so it must be an ordinal date.
                    var dayOfYear;
                    tokens = date.match(matchOrdinalDate);
                    if (tokens !== null) {

                        year = +tokens[1];
                        dayOfYear = +tokens[2];
                        inLeapYear = isLeapYear(year);

                        //This validation is only applicable for this format.
                                                if (dayOfYear < 1 || (inLeapYear && dayOfYear > 366) || (!inLeapYear && dayOfYear > 365)) {
                            throw new DeveloperError(iso8601ErrorMessage);
                        }
                                            } else {
                        tokens = date.match(matchWeekDate);
                        if (tokens !== null) {
                            //ISO week date to ordinal date from
                            //http://en.wikipedia.org/w/index.php?title=ISO_week_date&oldid=474176775
                            year = +tokens[1];
                            var weekNumber = +tokens[2];
                            var dayOfWeek = +tokens[3] || 0;

                                                        dashCount = date.split('-').length - 1;
                            if (dashCount > 0 &&
                               ((!defined(tokens[3]) && dashCount !== 1) ||
                               (defined(tokens[3]) && dashCount !== 2))) {
                                throw new DeveloperError(iso8601ErrorMessage);
                            }
                            
                            var january4 = new Date(Date.UTC(year, 0, 4));
                            dayOfYear = (weekNumber * 7) + dayOfWeek - january4.getUTCDay() - 3;
                        } else {
                            //None of our regular expressions succeeded in parsing the date properly.
                                                        throw new DeveloperError(iso8601ErrorMessage);
                                                    }
                    }
                    //Split an ordinal date into month/day.
                    tmp = new Date(Date.UTC(year, 0, 1));
                    tmp.setUTCDate(dayOfYear);
                    month = tmp.getUTCMonth() + 1;
                    day = tmp.getUTCDate();
                }
            }
        }

        //Now that we have all of the date components, validate them to make sure nothing is out of range.
        inLeapYear = isLeapYear(year);
                if (month < 1 || month > 12 || day < 1 || ((month !== 2 || !inLeapYear) && day > daysInMonth[month - 1]) || (inLeapYear && month === 2 && day > daysInLeapFeburary)) {
            throw new DeveloperError(iso8601ErrorMessage);
        }
        
        //Now move onto the time string, which is much simpler.
        //If no time is specified, it is considered the beginning of the day, UTC to match Javascript's implementation.
        var offsetIndex;
        if (defined(time)) {
            tokens = time.match(matchHoursMinutesSeconds);
            if (tokens !== null) {
                                dashCount = time.split(':').length - 1;
                if (dashCount > 0 && dashCount !== 2 && dashCount !== 3) {
                    throw new DeveloperError(iso8601ErrorMessage);
                }
                
                hour = +tokens[1];
                minute = +tokens[2];
                second = +tokens[3];
                millisecond = +(tokens[4] || 0) * 1000.0;
                offsetIndex = 5;
            } else {
                tokens = time.match(matchHoursMinutes);
                if (tokens !== null) {
                                        dashCount = time.split(':').length - 1;
                    if (dashCount > 2) {
                        throw new DeveloperError(iso8601ErrorMessage);
                    }
                    
                    hour = +tokens[1];
                    minute = +tokens[2];
                    second = +(tokens[3] || 0) * 60.0;
                    offsetIndex = 4;
                } else {
                    tokens = time.match(matchHours);
                    if (tokens !== null) {
                        hour = +tokens[1];
                        minute = +(tokens[2] || 0) * 60.0;
                        offsetIndex = 3;
                    } else {
                                                throw new DeveloperError(iso8601ErrorMessage);
                                            }
                }
            }

            //Validate that all values are in proper range.  Minutes and hours have special cases at 60 and 24.
                        if (minute >= 60 || second >= 61 || hour > 24 || (hour === 24 && (minute > 0 || second > 0 || millisecond > 0))) {
                throw new DeveloperError(iso8601ErrorMessage);
            }
            
            //Check the UTC offset value, if no value exists, use local time
            //a Z indicates UTC, + or - are offsets.
            var offset = tokens[offsetIndex];
            var offsetHours = +(tokens[offsetIndex + 1]);
            var offsetMinutes = +(tokens[offsetIndex + 2] || 0);
            switch (offset) {
            case '+':
                hour = hour - offsetHours;
                minute = minute - offsetMinutes;
                break;
            case '-':
                hour = hour + offsetHours;
                minute = minute + offsetMinutes;
                break;
            case 'Z':
                break;
            default:
                minute = minute + new Date(Date.UTC(year, month - 1, day, hour, minute)).getTimezoneOffset();
                break;
            }
        }

        //ISO8601 denotes a leap second by any time having a seconds component of 60 seconds.
        //If that's the case, we need to temporarily subtract a second in order to build a UTC date.
        //Then we add it back in after converting to TAI.
        var isLeapSecond = second === 60;
        if (isLeapSecond) {
            second--;
        }

        //Even if we successfully parsed the string into its components, after applying UTC offset or
        //special cases like 24:00:00 denoting midnight, we need to normalize the data appropriately.

        //milliseconds can never be greater than 1000, and seconds can't be above 60, so we start with minutes
        while (minute >= 60) {
            minute -= 60;
            hour++;
        }

        while (hour >= 24) {
            hour -= 24;
            day++;
        }

        tmp = (inLeapYear && month === 2) ? daysInLeapFeburary : daysInMonth[month - 1];
        while (day > tmp) {
            day -= tmp;
            month++;

            if (month > 12) {
                month -= 12;
                year++;
            }

            tmp = (inLeapYear && month === 2) ? daysInLeapFeburary : daysInMonth[month - 1];
        }

        //If UTC offset is at the beginning/end of the day, minutes can be negative.
        while (minute < 0) {
            minute += 60;
            hour--;
        }

        while (hour < 0) {
            hour += 24;
            day--;
        }

        while (day < 1) {
            month--;
            if (month < 1) {
                month += 12;
                year--;
            }

            tmp = (inLeapYear && month === 2) ? daysInLeapFeburary : daysInMonth[month - 1];
            day += tmp;
        }

        //Now create the JulianDate components from the Gregorian date and actually create our instance.
        var components = computeJulianDateComponents(year, month, day, hour, minute, second, millisecond);

        if (!defined(result)) {
            result = new JulianDate(components[0], components[1], TimeStandard.UTC);
        } else {
            setComponents(components[0], components[1], result);
            convertUtcToTai(result);
        }

        //If we were on a leap second, add it back.
        if (isLeapSecond) {
            JulianDate.addSeconds(result, 1, result);
        }

        return result;
    };

    /**
     * Creates a new instance that represents the current system time.
     * This is equivalent to calling <code>JulianDate.fromDate(new Date());</code>.
     *
     * @param {JulianDate} [result] An existing instance to use for the result.
     * @returns {JulianDate} The modified result parameter or a new instance if none was provided.
     */
    JulianDate.now = function(result) {
        return JulianDate.fromDate(new Date(), result);
    };

    var toGregorianDateScratch = new JulianDate(0, 0, TimeStandard.TAI);

    /**
     * Creates a {@link GregorianDate} from the provided instance.
     *
     * @param {JulianDate} julianDate The date to be converted.
     * @param {GregorianDate} [result] An existing instance to use for the result.
     * @returns {GregorianDate} The modified result parameter or a new instance if none was provided.
     */
    JulianDate.toGregorianDate = function(julianDate, result) {
                if (!defined(julianDate)) {
            throw new DeveloperError('julianDate is required.');
        }
        
        var isLeapSecond = false;
        var thisUtc = convertTaiToUtc(julianDate, toGregorianDateScratch);
        if (!defined(thisUtc)) {
            //Conversion to UTC will fail if we are during a leap second.
            //If that's the case, subtract a second and convert again.
            //JavaScript doesn't support leap seconds, so this results in second 59 being repeated twice.
            JulianDate.addSeconds(julianDate, -1, toGregorianDateScratch);
            thisUtc = convertTaiToUtc(toGregorianDateScratch, toGregorianDateScratch);
            isLeapSecond = true;
        }

        var julianDayNumber = thisUtc.dayNumber;
        var secondsOfDay = thisUtc.secondsOfDay;

        if (secondsOfDay >= 43200.0) {
            julianDayNumber += 1;
        }

        // Algorithm from page 604 of the Explanatory Supplement to the
        // Astronomical Almanac (Seidelmann 1992).
        var L = (julianDayNumber + 68569) | 0;
        var N = (4 * L / 146097) | 0;
        L = (L - (((146097 * N + 3) / 4) | 0)) | 0;
        var I = ((4000 * (L + 1)) / 1461001) | 0;
        L = (L - (((1461 * I) / 4) | 0) + 31) | 0;
        var J = ((80 * L) / 2447) | 0;
        var day = (L - (((2447 * J) / 80) | 0)) | 0;
        L = (J / 11) | 0;
        var month = (J + 2 - 12 * L) | 0;
        var year = (100 * (N - 49) + I + L) | 0;

        var hour = (secondsOfDay / TimeConstants.SECONDS_PER_HOUR) | 0;
        var remainingSeconds = secondsOfDay - (hour * TimeConstants.SECONDS_PER_HOUR);
        var minute = (remainingSeconds / TimeConstants.SECONDS_PER_MINUTE) | 0;
        remainingSeconds = remainingSeconds - (minute * TimeConstants.SECONDS_PER_MINUTE);
        var second = remainingSeconds | 0;
        var millisecond = ((remainingSeconds - second) / TimeConstants.SECONDS_PER_MILLISECOND);

        // JulianDates are noon-based
        hour += 12;
        if (hour > 23) {
            hour -= 24;
        }

        //If we were on a leap second, add it back.
        if (isLeapSecond) {
            second += 1;
        }

        if (!defined(result)) {
            return new GregorianDate(year, month, day, hour, minute, second, millisecond, isLeapSecond);
        }

        result.year = year;
        result.month = month;
        result.day = day;
        result.hour = hour;
        result.minute = minute;
        result.second = second;
        result.millisecond = millisecond;
        result.isLeapSecond = isLeapSecond;
        return result;
    };

    /**
     * Creates a JavaScript Date from the provided instance.
     * Since JavaScript dates are only accurate to the nearest millisecond and
     * cannot represent a leap second, consider using {@link JulianDate.toGregorianDate} instead.
     * If the provided JulianDate is during a leap second, the previous second is used.
     *
     * @param {JulianDate} julianDate The date to be converted.
     * @returns {Date} A new instance representing the provided date.
     */
    JulianDate.toDate = function(julianDate) {
                if (!defined(julianDate)) {
            throw new DeveloperError('julianDate is required.');
        }
        
        var gDate = JulianDate.toGregorianDate(julianDate, gregorianDateScratch);
        var second = gDate.second;
        if (gDate.isLeapSecond) {
            second -= 1;
        }
        return new Date(Date.UTC(gDate.year, gDate.month - 1, gDate.day, gDate.hour, gDate.minute, second, gDate.millisecond));
    };

    /**
     * Creates an ISO8601 representation of the provided date.
     *
     * @param {JulianDate} julianDate The date to be converted.
     * @param {Number} [precision] The number of fractional digits used to represent the seconds component.  By default, the most precise representation is used.
     * @returns {String} The ISO8601 representation of the provided date.
     */
    JulianDate.toIso8601 = function(julianDate, precision) {
                if (!defined(julianDate)) {
            throw new DeveloperError('julianDate is required.');
        }
        
        var gDate = JulianDate.toGregorianDate(julianDate, gregorianDateScratch);
        var year = gDate.year;
        var month = gDate.month;
        var day = gDate.day;
        var hour = gDate.hour;
        var minute = gDate.minute;
        var second = gDate.second;
        var millisecond = gDate.millisecond;

        // special case - Iso8601.MAXIMUM_VALUE produces a string which we can't parse unless we adjust.
        // 10000-01-01T00:00:00 is the same instant as 9999-12-31T24:00:00
        if (year === 10000 && month === 1 && day === 1 && hour === 0 && minute === 0 && second === 0 && millisecond === 0) {
            year = 9999;
            month = 12;
            day = 31;
            hour = 24;
        }

        var millisecondStr;

        if (!defined(precision) && millisecond !== 0) {
            //Forces milliseconds into a number with at least 3 digits to whatever the default toString() precision is.
            millisecondStr = (millisecond * 0.01).toString().replace('.', '');
            return sprintf('%04d-%02d-%02dT%02d:%02d:%02d.%sZ', year, month, day, hour, minute, second, millisecondStr);
        }

        //Precision is either 0 or milliseconds is 0 with undefined precision, in either case, leave off milliseconds entirely
        if (!defined(precision) || precision === 0) {
            return sprintf('%04d-%02d-%02dT%02d:%02d:%02dZ', year, month, day, hour, minute, second);
        }

        //Forces milliseconds into a number with at least 3 digits to whatever the specified precision is.
        millisecondStr = (millisecond * 0.01).toFixed(precision).replace('.', '').slice(0, precision);
        return sprintf('%04d-%02d-%02dT%02d:%02d:%02d.%sZ', year, month, day, hour, minute, second, millisecondStr);
    };

    /**
     * Duplicates a JulianDate instance.
     *
     * @param {JulianDate} julianDate The date to duplicate.
     * @param {JulianDate} [result] An existing instance to use for the result.
     * @returns {JulianDate} The modified result parameter or a new instance if none was provided. Returns undefined if julianDate is undefined.
     */
    JulianDate.clone = function(julianDate, result) {
        if (!defined(julianDate)) {
            return undefined;
        }
        if (!defined(result)) {
            return new JulianDate(julianDate.dayNumber, julianDate.secondsOfDay, TimeStandard.TAI);
        }
        result.dayNumber = julianDate.dayNumber;
        result.secondsOfDay = julianDate.secondsOfDay;
        return result;
    };

    /**
     * Compares two instances.
     *
     * @param {JulianDate} left The first instance.
     * @param {JulianDate} right The second instance.
     * @returns {Number} A negative value if left is less than right, a positive value if left is greater than right, or zero if left and right are equal.
     */
    JulianDate.compare = function(left, right) {
                if (!defined(left)) {
            throw new DeveloperError('left is required.');
        }
        if (!defined(right)) {
            throw new DeveloperError('right is required.');
        }
        
        var julianDayNumberDifference = left.dayNumber - right.dayNumber;
        if (julianDayNumberDifference !== 0) {
            return julianDayNumberDifference;
        }
        return left.secondsOfDay - right.secondsOfDay;
    };

    /**
     * Compares two instances and returns <code>true</code> if they are equal, <code>false</code> otherwise.
     *
     * @param {JulianDate} [left] The first instance.
     * @param {JulianDate} [right] The second instance.
     * @returns {Boolean} <code>true</code> if the dates are equal; otherwise, <code>false</code>.
     */
    JulianDate.equals = function(left, right) {
        return (left === right) ||
               (defined(left) &&
                defined(right) &&
                left.dayNumber === right.dayNumber &&
                left.secondsOfDay === right.secondsOfDay);
    };

    /**
     * Compares two instances and returns <code>true</code> if they are within <code>epsilon</code> seconds of
     * each other.  That is, in order for the dates to be considered equal (and for
     * this function to return <code>true</code>), the absolute value of the difference between them, in
     * seconds, must be less than <code>epsilon</code>.
     *
     * @param {JulianDate} [left] The first instance.
     * @param {JulianDate} [right] The second instance.
     * @param {Number} epsilon The maximum number of seconds that should separate the two instances.
     * @returns {Boolean} <code>true</code> if the two dates are within <code>epsilon</code> seconds of each other; otherwise <code>false</code>.
     */
    JulianDate.equalsEpsilon = function(left, right, epsilon) {
                if (!defined(epsilon)) {
            throw new DeveloperError('epsilon is required.');
        }
        
        return (left === right) ||
               (defined(left) &&
                defined(right) &&
                Math.abs(JulianDate.secondsDifference(left, right)) <= epsilon);
    };

    /**
     * Computes the total number of whole and fractional days represented by the provided instance.
     *
     * @param {JulianDate} julianDate The date.
     * @returns {Number} The Julian date as single floating point number.
     */
    JulianDate.totalDays = function(julianDate) {
                if (!defined(julianDate)) {
            throw new DeveloperError('julianDate is required.');
        }
                return julianDate.dayNumber + (julianDate.secondsOfDay / TimeConstants.SECONDS_PER_DAY);
    };

    /**
     * Computes the difference in seconds between the provided instance.
     *
     * @param {JulianDate} left The first instance.
     * @param {JulianDate} right The second instance.
     * @returns {Number} The difference, in seconds, when subtracting <code>right</code> from <code>left</code>.
     */
    JulianDate.secondsDifference = function(left, right) {
                if (!defined(left)) {
            throw new DeveloperError('left is required.');
        }
        if (!defined(right)) {
            throw new DeveloperError('right is required.');
        }
        
        var dayDifference = (left.dayNumber - right.dayNumber) * TimeConstants.SECONDS_PER_DAY;
        return (dayDifference + (left.secondsOfDay - right.secondsOfDay));
    };

    /**
     * Computes the difference in days between the provided instance.
     *
     * @param {JulianDate} left The first instance.
     * @param {JulianDate} right The second instance.
     * @returns {Number} The difference, in days, when subtracting <code>right</code> from <code>left</code>.
     */
    JulianDate.daysDifference = function(left, right) {
                if (!defined(left)) {
            throw new DeveloperError('left is required.');
        }
        if (!defined(right)) {
            throw new DeveloperError('right is required.');
        }
        
        var dayDifference = (left.dayNumber - right.dayNumber);
        var secondDifference = (left.secondsOfDay - right.secondsOfDay) / TimeConstants.SECONDS_PER_DAY;
        return dayDifference + secondDifference;
    };

    /**
     * Computes the number of seconds the provided instance is ahead of UTC.
     *
     * @param {JulianDate} julianDate The date.
     * @returns {Number} The number of seconds the provided instance is ahead of UTC
     */
    JulianDate.computeTaiMinusUtc = function(julianDate) {
        binarySearchScratchLeapSecond.julianDate = julianDate;
        var leapSeconds = JulianDate.leapSeconds;
        var index = binarySearch(leapSeconds, binarySearchScratchLeapSecond, compareLeapSecondDates);
        if (index < 0) {
            index = ~index;
            --index;
            if (index < 0) {
                index = 0;
            }
        }
        return leapSeconds[index].offset;
    };

    /**
     * Adds the provided number of seconds to the provided date instance.
     *
     * @param {JulianDate} julianDate The date.
     * @param {Number} seconds The number of seconds to add or subtract.
     * @param {JulianDate} result An existing instance to use for the result.
     * @returns {JulianDate} The modified result parameter.
     */
    JulianDate.addSeconds = function(julianDate, seconds, result) {
                if (!defined(julianDate)) {
            throw new DeveloperError('julianDate is required.');
        }
        if (!defined(seconds)) {
            throw new DeveloperError('seconds is required.');
        }
        if (!defined(result)) {
            throw new DeveloperError('result is required.');
        }
        
        return setComponents(julianDate.dayNumber, julianDate.secondsOfDay + seconds, result);
    };

    /**
     * Adds the provided number of minutes to the provided date instance.
     *
     * @param {JulianDate} julianDate The date.
     * @param {Number} minutes The number of minutes to add or subtract.
     * @param {JulianDate} result An existing instance to use for the result.
     * @returns {JulianDate} The modified result parameter.
     */
    JulianDate.addMinutes = function(julianDate, minutes, result) {
                if (!defined(julianDate)) {
            throw new DeveloperError('julianDate is required.');
        }
        if (!defined(minutes)) {
            throw new DeveloperError('minutes is required.');
        }
        if (!defined(result)) {
            throw new DeveloperError('result is required.');
        }
        
        var newSecondsOfDay = julianDate.secondsOfDay + (minutes * TimeConstants.SECONDS_PER_MINUTE);
        return setComponents(julianDate.dayNumber, newSecondsOfDay, result);
    };

    /**
     * Adds the provided number of hours to the provided date instance.
     *
     * @param {JulianDate} julianDate The date.
     * @param {Number} hours The number of hours to add or subtract.
     * @param {JulianDate} result An existing instance to use for the result.
     * @returns {JulianDate} The modified result parameter.
     */
    JulianDate.addHours = function(julianDate, hours, result) {
                if (!defined(julianDate)) {
            throw new DeveloperError('julianDate is required.');
        }
        if (!defined(hours)) {
            throw new DeveloperError('hours is required.');
        }
        if (!defined(result)) {
            throw new DeveloperError('result is required.');
        }
        
        var newSecondsOfDay = julianDate.secondsOfDay + (hours * TimeConstants.SECONDS_PER_HOUR);
        return setComponents(julianDate.dayNumber, newSecondsOfDay, result);
    };

    /**
     * Adds the provided number of days to the provided date instance.
     *
     * @param {JulianDate} julianDate The date.
     * @param {Number} days The number of days to add or subtract.
     * @param {JulianDate} result An existing instance to use for the result.
     * @returns {JulianDate} The modified result parameter.
     */
    JulianDate.addDays = function(julianDate, days, result) {
                if (!defined(julianDate)) {
            throw new DeveloperError('julianDate is required.');
        }
        if (!defined(days)) {
            throw new DeveloperError('days is required.');
        }
        if (!defined(result)) {
            throw new DeveloperError('result is required.');
        }
        
        var newJulianDayNumber = julianDate.dayNumber + days;
        return setComponents(newJulianDayNumber, julianDate.secondsOfDay, result);
    };

    /**
     * Compares the provided instances and returns <code>true</code> if <code>left</code> is earlier than <code>right</code>, <code>false</code> otherwise.
     *
     * @param {JulianDate} left The first instance.
     * @param {JulianDate} right The second instance.
     * @returns {Boolean} <code>true</code> if <code>left</code> is earlier than <code>right</code>, <code>false</code> otherwise.
     */
    JulianDate.lessThan = function(left, right) {
        return JulianDate.compare(left, right) < 0;
    };

    /**
     * Compares the provided instances and returns <code>true</code> if <code>left</code> is earlier than or equal to <code>right</code>, <code>false</code> otherwise.
     *
     * @param {JulianDate} left The first instance.
     * @param {JulianDate} right The second instance.
     * @returns {Boolean} <code>true</code> if <code>left</code> is earlier than or equal to <code>right</code>, <code>false</code> otherwise.
     */
    JulianDate.lessThanOrEquals = function(left, right) {
        return JulianDate.compare(left, right) <= 0;
    };

    /**
     * Compares the provided instances and returns <code>true</code> if <code>left</code> is later than <code>right</code>, <code>false</code> otherwise.
     *
     * @param {JulianDate} left The first instance.
     * @param {JulianDate} right The second instance.
     * @returns {Boolean} <code>true</code> if <code>left</code> is later than <code>right</code>, <code>false</code> otherwise.
     */
    JulianDate.greaterThan = function(left, right) {
        return JulianDate.compare(left, right) > 0;
    };

    /**
     * Compares the provided instances and returns <code>true</code> if <code>left</code> is later than or equal to <code>right</code>, <code>false</code> otherwise.
     *
     * @param {JulianDate} left The first instance.
     * @param {JulianDate} right The second instance.
     * @returns {Boolean} <code>true</code> if <code>left</code> is later than or equal to <code>right</code>, <code>false</code> otherwise.
     */
    JulianDate.greaterThanOrEquals = function(left, right) {
        return JulianDate.compare(left, right) >= 0;
    };

    /**
     * Duplicates this instance.
     *
     * @param {JulianDate} [result] An existing instance to use for the result.
     * @returns {JulianDate} The modified result parameter or a new instance if none was provided.
     */
    JulianDate.prototype.clone = function(result) {
        return JulianDate.clone(this, result);
    };

    /**
     * Compares this and the provided instance and returns <code>true</code> if they are equal, <code>false</code> otherwise.
     *
     * @param {JulianDate} [right] The second instance.
     * @returns {Boolean} <code>true</code> if the dates are equal; otherwise, <code>false</code>.
     */
    JulianDate.prototype.equals = function(right) {
        return JulianDate.equals(this, right);
    };

    /**
     * Compares this and the provided instance and returns <code>true</code> if they are within <code>epsilon</code> seconds of
     * each other.  That is, in order for the dates to be considered equal (and for
     * this function to return <code>true</code>), the absolute value of the difference between them, in
     * seconds, must be less than <code>epsilon</code>.
     *
     * @param {JulianDate} [right] The second instance.
     * @param {Number} epsilon The maximum number of seconds that should separate the two instances.
     * @returns {Boolean} <code>true</code> if the two dates are within <code>epsilon</code> seconds of each other; otherwise <code>false</code>.
     */
    JulianDate.prototype.equalsEpsilon = function(right, epsilon) {
        return JulianDate.equalsEpsilon(this, right, epsilon);
    };

    /**
     * Creates a string representing this date in ISO8601 format.
     *
     * @returns {String} A string representing this date in ISO8601 format.
     */
    JulianDate.prototype.toString = function() {
        return JulianDate.toIso8601(this);
    };

    /**
     * Gets or sets the list of leap seconds used throughout Cesium.
     * @memberof JulianDate
     * @type {LeapSecond[]}
     */
    JulianDate.leapSeconds = [
                               new LeapSecond(new JulianDate(2441317, 43210.0, TimeStandard.TAI), 10), // January 1, 1972 00:00:00 UTC
                               new LeapSecond(new JulianDate(2441499, 43211.0, TimeStandard.TAI), 11), // July 1, 1972 00:00:00 UTC
                               new LeapSecond(new JulianDate(2441683, 43212.0, TimeStandard.TAI), 12), // January 1, 1973 00:00:00 UTC
                               new LeapSecond(new JulianDate(2442048, 43213.0, TimeStandard.TAI), 13), // January 1, 1974 00:00:00 UTC
                               new LeapSecond(new JulianDate(2442413, 43214.0, TimeStandard.TAI), 14), // January 1, 1975 00:00:00 UTC
                               new LeapSecond(new JulianDate(2442778, 43215.0, TimeStandard.TAI), 15), // January 1, 1976 00:00:00 UTC
                               new LeapSecond(new JulianDate(2443144, 43216.0, TimeStandard.TAI), 16), // January 1, 1977 00:00:00 UTC
                               new LeapSecond(new JulianDate(2443509, 43217.0, TimeStandard.TAI), 17), // January 1, 1978 00:00:00 UTC
                               new LeapSecond(new JulianDate(2443874, 43218.0, TimeStandard.TAI), 18), // January 1, 1979 00:00:00 UTC
                               new LeapSecond(new JulianDate(2444239, 43219.0, TimeStandard.TAI), 19), // January 1, 1980 00:00:00 UTC
                               new LeapSecond(new JulianDate(2444786, 43220.0, TimeStandard.TAI), 20), // July 1, 1981 00:00:00 UTC
                               new LeapSecond(new JulianDate(2445151, 43221.0, TimeStandard.TAI), 21), // July 1, 1982 00:00:00 UTC
                               new LeapSecond(new JulianDate(2445516, 43222.0, TimeStandard.TAI), 22), // July 1, 1983 00:00:00 UTC
                               new LeapSecond(new JulianDate(2446247, 43223.0, TimeStandard.TAI), 23), // July 1, 1985 00:00:00 UTC
                               new LeapSecond(new JulianDate(2447161, 43224.0, TimeStandard.TAI), 24), // January 1, 1988 00:00:00 UTC
                               new LeapSecond(new JulianDate(2447892, 43225.0, TimeStandard.TAI), 25), // January 1, 1990 00:00:00 UTC
                               new LeapSecond(new JulianDate(2448257, 43226.0, TimeStandard.TAI), 26), // January 1, 1991 00:00:00 UTC
                               new LeapSecond(new JulianDate(2448804, 43227.0, TimeStandard.TAI), 27), // July 1, 1992 00:00:00 UTC
                               new LeapSecond(new JulianDate(2449169, 43228.0, TimeStandard.TAI), 28), // July 1, 1993 00:00:00 UTC
                               new LeapSecond(new JulianDate(2449534, 43229.0, TimeStandard.TAI), 29), // July 1, 1994 00:00:00 UTC
                               new LeapSecond(new JulianDate(2450083, 43230.0, TimeStandard.TAI), 30), // January 1, 1996 00:00:00 UTC
                               new LeapSecond(new JulianDate(2450630, 43231.0, TimeStandard.TAI), 31), // July 1, 1997 00:00:00 UTC
                               new LeapSecond(new JulianDate(2451179, 43232.0, TimeStandard.TAI), 32), // January 1, 1999 00:00:00 UTC
                               new LeapSecond(new JulianDate(2453736, 43233.0, TimeStandard.TAI), 33), // January 1, 2006 00:00:00 UTC
                               new LeapSecond(new JulianDate(2454832, 43234.0, TimeStandard.TAI), 34), // January 1, 2009 00:00:00 UTC
                               new LeapSecond(new JulianDate(2456109, 43235.0, TimeStandard.TAI), 35), // July 1, 2012 00:00:00 UTC
                               new LeapSecond(new JulianDate(2457204, 43236.0, TimeStandard.TAI), 36), // July 1, 2015 00:00:00 UTC
                               new LeapSecond(new JulianDate(2457754, 43237.0, TimeStandard.TAI), 37)  // January 1, 2017 00:00:00 UTC
                             ];

    return JulianDate;
});

/**
 * @license
 *
 * Grauw URI utilities
 *
 * See: http://hg.grauw.nl/grauw-lib/file/tip/src/uri.js
 *
 * @author Laurens Holst (http://www.grauw.nl/)
 *
 *   Copyright 2012 Laurens Holst
 *
 *   Licensed under the Apache License, Version 2.0 (the "License");
 *   you may not use this file except in compliance with the License.
 *   You may obtain a copy of the License at
 *
 *       http://www.apache.org/licenses/LICENSE-2.0
 *
 *   Unless required by applicable law or agreed to in writing, software
 *   distributed under the License is distributed on an "AS IS" BASIS,
 *   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *   See the License for the specific language governing permissions and
 *   limitations under the License.
 *
 */
define('ThirdParty/Uri',[],function() {

	/**
	 * Constructs a URI object.
	 * @constructor
	 * @class Implementation of URI parsing and base URI resolving algorithm in RFC 3986.
	 * @param {string|URI} uri A string or URI object to create the object from.
	 */
	function URI(uri) {
		if (uri instanceof URI) {  // copy constructor
			this.scheme = uri.scheme;
			this.authority = uri.authority;
			this.path = uri.path;
			this.query = uri.query;
			this.fragment = uri.fragment;
		} else if (uri) {  // uri is URI string or cast to string
			var c = parseRegex.exec(uri);
			this.scheme = c[1];
			this.authority = c[2];
			this.path = c[3];
			this.query = c[4];
			this.fragment = c[5];
		}
	}
	// Initial values on the prototype
	URI.prototype.scheme    = null;
	URI.prototype.authority = null;
	URI.prototype.path      = '';
	URI.prototype.query     = null;
	URI.prototype.fragment  = null;

	// Regular expression from RFC 3986 appendix B
	var parseRegex = new RegExp('^(?:([^:/?#]+):)?(?://([^/?#]*))?([^?#]*)(?:\\?([^#]*))?(?:#(.*))?$');

	/**
	 * Returns the scheme part of the URI.
	 * In "http://example.com:80/a/b?x#y" this is "http".
	 */
	URI.prototype.getScheme = function() {
		return this.scheme;
	};

	/**
	 * Returns the authority part of the URI.
	 * In "http://example.com:80/a/b?x#y" this is "example.com:80".
	 */
	URI.prototype.getAuthority = function() {
		return this.authority;
	};

	/**
	 * Returns the path part of the URI.
	 * In "http://example.com:80/a/b?x#y" this is "/a/b".
	 * In "mailto:mike@example.com" this is "mike@example.com".
	 */
	URI.prototype.getPath = function() {
		return this.path;
	};

	/**
	 * Returns the query part of the URI.
	 * In "http://example.com:80/a/b?x#y" this is "x".
	 */
	URI.prototype.getQuery = function() {
		return this.query;
	};

	/**
	 * Returns the fragment part of the URI.
	 * In "http://example.com:80/a/b?x#y" this is "y".
	 */
	URI.prototype.getFragment = function() {
		return this.fragment;
	};

	/**
	 * Tests whether the URI is an absolute URI.
	 * See RFC 3986 section 4.3.
	 */
	URI.prototype.isAbsolute = function() {
		return !!this.scheme && !this.fragment;
	};

	///**
	//* Extensive validation of the URI against the ABNF in RFC 3986
	//*/
	//URI.prototype.validate

	/**
	 * Tests whether the URI is a same-document reference.
	 * See RFC 3986 section 4.4.
	 *
	 * To perform more thorough comparison, you can normalise the URI objects.
	 */
	URI.prototype.isSameDocumentAs = function(uri) {
		return uri.scheme == this.scheme &&
		    uri.authority == this.authority &&
		         uri.path == this.path &&
		        uri.query == this.query;
	};

	/**
	 * Simple String Comparison of two URIs.
	 * See RFC 3986 section 6.2.1.
	 *
	 * To perform more thorough comparison, you can normalise the URI objects.
	 */
	URI.prototype.equals = function(uri) {
		return this.isSameDocumentAs(uri) && uri.fragment == this.fragment;
	};

	/**
	 * Normalizes the URI using syntax-based normalization.
	 * This includes case normalization, percent-encoding normalization and path segment normalization.
	 * XXX: Percent-encoding normalization does not escape characters that need to be escaped.
	 *      (Although that would not be a valid URI in the first place. See validate().)
	 * See RFC 3986 section 6.2.2.
	 */
	URI.prototype.normalize = function() {
		this.removeDotSegments();
		if (this.scheme)
			this.scheme = this.scheme.toLowerCase();
		if (this.authority)
			this.authority = this.authority.replace(authorityRegex, replaceAuthority).
									replace(caseRegex, replaceCase);
		if (this.path)
			this.path = this.path.replace(caseRegex, replaceCase);
		if (this.query)
			this.query = this.query.replace(caseRegex, replaceCase);
		if (this.fragment)
			this.fragment = this.fragment.replace(caseRegex, replaceCase);
	};

	var caseRegex = /%[0-9a-z]{2}/gi;
	var percentRegex = /[a-zA-Z0-9\-\._~]/;
	var authorityRegex = /(.*@)?([^@:]*)(:.*)?/;

	function replaceCase(str) {
		var dec = unescape(str);
		return percentRegex.test(dec) ? dec : str.toUpperCase();
	}

	function replaceAuthority(str, p1, p2, p3) {
		return (p1 || '') + p2.toLowerCase() + (p3 || '');
	}

	/**
	 * Resolve a relative URI (this) against a base URI.
	 * The base URI must be an absolute URI.
	 * See RFC 3986 section 5.2
	 */
	URI.prototype.resolve = function(baseURI) {
		var uri = new URI();
		if (this.scheme) {
			uri.scheme = this.scheme;
			uri.authority = this.authority;
			uri.path = this.path;
			uri.query = this.query;
		} else {
			uri.scheme = baseURI.scheme;
			if (this.authority) {
				uri.authority = this.authority;
				uri.path = this.path;
				uri.query = this.query;
			} else {
				uri.authority = baseURI.authority;
				if (this.path == '') {
					uri.path = baseURI.path;
					uri.query = this.query || baseURI.query;
				} else {
					if (this.path.charAt(0) == '/') {
						uri.path = this.path;
						uri.removeDotSegments();
					} else {
						if (baseURI.authority && baseURI.path == '') {
							uri.path = '/' + this.path;
						} else {
							uri.path = baseURI.path.substring(0, baseURI.path.lastIndexOf('/') + 1) + this.path;
						}
						uri.removeDotSegments();
					}
					uri.query = this.query;
				}
			}
		}
		uri.fragment = this.fragment;
		return uri;
	};

	/**
	 * Remove dot segments from path.
	 * See RFC 3986 section 5.2.4
	 * @private
	 */
	URI.prototype.removeDotSegments = function() {
		var input = this.path.split('/'),
			output = [],
			segment,
			absPath = input[0] == '';
		if (absPath)
			input.shift();
		var sFirst = input[0] == '' ? input.shift() : null;
		while (input.length) {
			segment = input.shift();
			if (segment == '..') {
				output.pop();
			} else if (segment != '.') {
				output.push(segment);
			}
		}
		if (segment == '.' || segment == '..')
			output.push('');
		if (absPath)
			output.unshift('');
		this.path = output.join('/');
	};

	// We don't like this function because it builds up a cache that is never cleared.
//	/**
//	 * Resolves a relative URI against an absolute base URI.
//	 * Convenience method.
//	 * @param {String} uri the relative URI to resolve
//	 * @param {String} baseURI the base URI (must be absolute) to resolve against
//	 */
//	URI.resolve = function(sURI, sBaseURI) {
//		var uri = cache[sURI] || (cache[sURI] = new URI(sURI));
//		var baseURI = cache[sBaseURI] || (cache[sBaseURI] = new URI(sBaseURI));
//		return uri.resolve(baseURI).toString();
//	};

//	var cache = {};

	/**
	 * Serialises the URI to a string.
	 */
	URI.prototype.toString = function() {
		var result = '';
		if (this.scheme)
			result += this.scheme + ':';
		if (this.authority)
			result += '//' + this.authority;
		result += this.path;
		if (this.query)
			result += '?' + this.query;
		if (this.fragment)
			result += '#' + this.fragment;
		return result;
	};

return URI;
});

define('Core/appendForwardSlash',[],function() {
    'use strict';

    /**
     * @private
     */
    function appendForwardSlash(url) {
        if (url.length === 0 || url[url.length - 1] !== '/') {
            url = url + '/';
        }
        return url;
    }

    return appendForwardSlash;
});

define('Core/clone',[
        './defaultValue'
    ], function(
        defaultValue) {
    'use strict';

    /**
     * Clones an object, returning a new object containing the same properties.
     *
     * @exports clone
     *
     * @param {Object} object The object to clone.
     * @param {Boolean} [deep=false] If true, all properties will be deep cloned recursively.
     * @returns {Object} The cloned object.
     */
    function clone(object, deep) {
        if (object === null || typeof object !== 'object') {
            return object;
        }

        deep = defaultValue(deep, false);

        var result = new object.constructor();
        for ( var propertyName in object) {
            if (object.hasOwnProperty(propertyName)) {
                var value = object[propertyName];
                if (deep) {
                    value = clone(value, deep);
                }
                result[propertyName] = value;
            }
        }

        return result;
    }

    return clone;
});

define('Core/combine',[
        './defaultValue',
        './defined'
    ], function(
        defaultValue,
        defined) {
    'use strict';

    /**
     * Merges two objects, copying their properties onto a new combined object. When two objects have the same
     * property, the value of the property on the first object is used.  If either object is undefined,
     * it will be treated as an empty object.
     *
     * @example
     * var object1 = {
     *     propOne : 1,
     *     propTwo : {
     *         value1 : 10
     *     }
     * }
     * var object2 = {
     *     propTwo : 2
     * }
     * var final = Cesium.combine(object1, object2);
     *
     * // final === {
     * //     propOne : 1,
     * //     propTwo : {
     * //         value1 : 10
     * //     }
     * // }
     *
     * @param {Object} [object1] The first object to merge.
     * @param {Object} [object2] The second object to merge.
     * @param {Boolean} [deep=false] Perform a recursive merge.
     * @returns {Object} The combined object containing all properties from both objects.
     *
     * @exports combine
     */
    function combine(object1, object2, deep) {
        deep = defaultValue(deep, false);

        var result = {};

        var object1Defined = defined(object1);
        var object2Defined = defined(object2);
        var property;
        var object1Value;
        var object2Value;
        if (object1Defined) {
            for (property in object1) {
                if (object1.hasOwnProperty(property)) {
                    object1Value = object1[property];
                    if (object2Defined && deep && typeof object1Value === 'object' && object2.hasOwnProperty(property)) {
                        object2Value = object2[property];
                        if (typeof object2Value === 'object') {
                            result[property] = combine(object1Value, object2Value, deep);
                        } else {
                            result[property] = object1Value;
                        }
                    } else {
                        result[property] = object1Value;
                    }
                }
            }
        }
        if (object2Defined) {
            for (property in object2) {
                if (object2.hasOwnProperty(property) && !result.hasOwnProperty(property)) {
                    object2Value = object2[property];
                    result[property] = object2Value;
                }
            }
        }
        return result;
    }

    return combine;
});

define('Core/getAbsoluteUri',[
        '../ThirdParty/Uri',
        './defaultValue',
        './defined',
        './DeveloperError'
    ], function(
        Uri,
        defaultValue,
        defined,
        DeveloperError) {
    'use strict';

    /**
     * Given a relative Uri and a base Uri, returns the absolute Uri of the relative Uri.
     * @exports getAbsoluteUri
     *
     * @param {String} relative The relative Uri.
     * @param {String} [base] The base Uri.
     * @returns {String} The absolute Uri of the given relative Uri.
     *
     * @example
     * //absolute Uri will be "https://test.com/awesome.png";
     * var absoluteUri = Cesium.getAbsoluteUri('awesome.png', 'https://test.com');
     */
    function getAbsoluteUri(relative, base) {
        var documentObject;
        if (typeof document !== 'undefined') {
            documentObject = document;
        }

        return getAbsoluteUri._implementation(relative, base, documentObject);
    }

    getAbsoluteUri._implementation = function(relative, base, documentObject) {
                if (!defined(relative)) {
            throw new DeveloperError('relative uri is required.');
        }
        
        if (!defined(base)) {
            if (typeof documentObject === 'undefined') {
                return relative;
            }
            base = defaultValue(documentObject.baseURI, documentObject.location.href);
        }

        var baseUri = new Uri(base);
        var relativeUri = new Uri(relative);
        return relativeUri.resolve(baseUri).toString();
    };

    return getAbsoluteUri;
});

define('Core/getBaseUri',[
        '../ThirdParty/Uri',
        './defined',
        './DeveloperError'
    ], function(
        Uri,
        defined,
        DeveloperError) {
    'use strict';

    /**
     * Given a URI, returns the base path of the URI.
     * @exports getBaseUri
     *
     * @param {String} uri The Uri.
     * @param {Boolean} [includeQuery = false] Whether or not to include the query string and fragment form the uri
     * @returns {String} The base path of the Uri.
     *
     * @example
     * // basePath will be "/Gallery/";
     * var basePath = Cesium.getBaseUri('/Gallery/simple.czml?value=true&example=false');
     *
     * // basePath will be "/Gallery/?value=true&example=false";
     * var basePath = Cesium.getBaseUri('/Gallery/simple.czml?value=true&example=false', true);
     */
    function getBaseUri(uri, includeQuery) {
                if (!defined(uri)) {
            throw new DeveloperError('uri is required.');
        }
        
        var basePath = '';
        var i = uri.lastIndexOf('/');
        if (i !== -1) {
            basePath = uri.substring(0, i + 1);
        }

        if (!includeQuery) {
            return basePath;
        }

        uri = new Uri(uri);
        if (defined(uri.query)) {
            basePath += '?' + uri.query;
        }
        if (defined(uri.fragment)){
            basePath += '#' + uri.fragment;
        }

        return basePath;
    }

    return getBaseUri;
});

define('Core/getExtensionFromUri',[
        '../ThirdParty/Uri',
        './defined',
        './DeveloperError'
    ], function(
        Uri,
        defined,
        DeveloperError) {
    'use strict';

    /**
     * Given a URI, returns the extension of the URI.
     * @exports getExtensionFromUri
     *
     * @param {String} uri The Uri.
     * @returns {String} The extension of the Uri.
     *
     * @example
     * //extension will be "czml";
     * var extension = Cesium.getExtensionFromUri('/Gallery/simple.czml?value=true&example=false');
     */
    function getExtensionFromUri(uri) {
                if (!defined(uri)) {
            throw new DeveloperError('uri is required.');
        }
        
        var uriObject = new Uri(uri);
        uriObject.normalize();
        var path = uriObject.path;
        var index = path.lastIndexOf('/');
        if (index !== -1) {
            path = path.substr(index + 1);
        }
        index = path.lastIndexOf('.');
        if (index === -1) {
            path = '';
        } else {
            path = path.substr(index + 1);
        }
        return path;
    }

    return getExtensionFromUri;
});

define('Core/isBlobUri',[
        './Check'
    ], function(
        Check) {
    'use strict';

    var blobUriRegex = /^blob:/i;

    /**
     * Determines if the specified uri is a blob uri.
     *
     * @exports isBlobUri
     *
     * @param {String} uri The uri to test.
     * @returns {Boolean} true when the uri is a blob uri; otherwise, false.
     *
     * @private
     */
    function isBlobUri(uri) {
                Check.typeOf.string('uri', uri);
        
        return blobUriRegex.test(uri);
    }

    return isBlobUri;
});

define('Core/isCrossOriginUrl',[
        './defined'
    ], function(
        defined) {
    'use strict';

    var a;

    /**
     * Given a URL, determine whether that URL is considered cross-origin to the current page.
     *
     * @private
     */
    function isCrossOriginUrl(url) {
        if (!defined(a)) {
            a = document.createElement('a');
        }

        // copy window location into the anchor to get consistent results
        // when the port is default for the protocol (e.g. 80 for HTTP)
        a.href = window.location.href;

        // host includes both hostname and port if the port is not standard
        var host = a.host;
        var protocol = a.protocol;

        a.href = url;
        // IE only absolutizes href on get, not set
        a.href = a.href; // eslint-disable-line no-self-assign

        return protocol !== a.protocol || host !== a.host;
    }

    return isCrossOriginUrl;
});

define('Core/isDataUri',[
        './Check'
    ], function(
        Check) {
    'use strict';

    var dataUriRegex = /^data:/i;

    /**
     * Determines if the specified uri is a data uri.
     *
     * @exports isDataUri
     *
     * @param {String} uri The uri to test.
     * @returns {Boolean} true when the uri is a data uri; otherwise, false.
     *
     * @private
     */
    function isDataUri(uri) {
                Check.typeOf.string('uri', uri);
        
        return dataUriRegex.test(uri);
    }

    return isDataUri;
});

define('Core/loadAndExecuteScript',[
    '../ThirdParty/when'
], function(
    when) {
        'use strict';

    /**
     * @private
     */
    function loadAndExecuteScript(url) {
        var deferred = when.defer();
        var script = document.createElement('script');
        script.async = true;
        script.src = url;

        var head = document.getElementsByTagName('head')[0];
        script.onload = function() {
            script.onload = undefined;
            head.removeChild(script);
            deferred.resolve();
        };
        script.onerror = function(e) {
            deferred.reject(e);
        };

        head.appendChild(script);

        return deferred.promise;
    }

    return loadAndExecuteScript;
});

define('Core/isArray',[
        './defined'
    ], function(
        defined) {
    'use strict';

    /**
     * Tests an object to see if it is an array.
     * @exports isArray
     *
     * @param {*} value The value to test.
     * @returns {Boolean} true if the value is an array, false otherwise.
     */
    var isArray = Array.isArray;
    if (!defined(isArray)) {
        isArray = function(value) {
            return Object.prototype.toString.call(value) === '[object Array]';
        };
    }

    return isArray;
});

define('Core/objectToQuery',[
        './defined',
        './DeveloperError',
        './isArray'
    ], function(
        defined,
        DeveloperError,
        isArray) {
    'use strict';

    /**
     * Converts an object representing a set of name/value pairs into a query string,
     * with names and values encoded properly for use in a URL.  Values that are arrays
     * will produce multiple values with the same name.
     * @exports objectToQuery
     *
     * @param {Object} obj The object containing data to encode.
     * @returns {String} An encoded query string.
     *
     *
     * @example
     * var str = Cesium.objectToQuery({
     *     key1 : 'some value',
     *     key2 : 'a/b',
     *     key3 : ['x', 'y']
     * });
     *
     * @see queryToObject
     * // str will be:
     * // 'key1=some%20value&key2=a%2Fb&key3=x&key3=y'
     */
    function objectToQuery(obj) {
                if (!defined(obj)) {
            throw new DeveloperError('obj is required.');
        }
        
        var result = '';
        for ( var propName in obj) {
            if (obj.hasOwnProperty(propName)) {
                var value = obj[propName];

                var part = encodeURIComponent(propName) + '=';
                if (isArray(value)) {
                    for (var i = 0, len = value.length; i < len; ++i) {
                        result += part + encodeURIComponent(value[i]) + '&';
                    }
                } else {
                    result += part + encodeURIComponent(value) + '&';
                }
            }
        }

        // trim last &
        result = result.slice(0, -1);

        // This function used to replace %20 with + which is more compact and readable.
        // However, some servers didn't properly handle + as a space.
        // https://github.com/AnalyticalGraphicsInc/cesium/issues/2192

        return result;
    }

    return objectToQuery;
});

define('Core/queryToObject',[
        './defined',
        './DeveloperError',
        './isArray'
    ], function(
        defined,
        DeveloperError,
        isArray) {
    'use strict';

    /**
     * Parses a query string into an object, where the keys and values of the object are the
     * name/value pairs from the query string, decoded. If a name appears multiple times,
     * the value in the object will be an array of values.
     * @exports queryToObject
     *
     * @param {String} queryString The query string.
     * @returns {Object} An object containing the parameters parsed from the query string.
     *
     *
     * @example
     * var obj = Cesium.queryToObject('key1=some%20value&key2=a%2Fb&key3=x&key3=y');
     * // obj will be:
     * // {
     * //   key1 : 'some value',
     * //   key2 : 'a/b',
     * //   key3 : ['x', 'y']
     * // }
     *
     * @see objectToQuery
     */
    function queryToObject(queryString) {
                if (!defined(queryString)) {
            throw new DeveloperError('queryString is required.');
        }
        
        var result = {};
        if (queryString === '') {
            return result;
        }
        var parts = queryString.replace(/\+/g, '%20').split(/[&;]/);
        for (var i = 0, len = parts.length; i < len; ++i) {
            var subparts = parts[i].split('=');

            var name = decodeURIComponent(subparts[0]);
            var value = subparts[1];
            if (defined(value)) {
                value = decodeURIComponent(value);
            } else {
                value = '';
            }

            var resultValue = result[name];
            if (typeof resultValue === 'string') {
                // expand the single value to an array
                result[name] = [resultValue, value];
            } else if (isArray(resultValue)) {
                resultValue.push(value);
            } else {
                result[name] = value;
            }
        }
        return result;
    }

    return queryToObject;
});

define('Core/RequestState',[
        '../Core/freezeObject'
    ], function(
        freezeObject) {
    'use strict';

    /**
     * State of the request.
     *
     * @exports RequestState
     */
    var RequestState = {
        /**
         * Initial unissued state.
         *
         * @type Number
         * @constant
         */
        UNISSUED : 0,

        /**
         * Issued but not yet active. Will become active when open slots are available.
         *
         * @type Number
         * @constant
         */
        ISSUED : 1,

        /**
         * Actual http request has been sent.
         *
         * @type Number
         * @constant
         */
        ACTIVE : 2,

        /**
         * Request completed successfully.
         *
         * @type Number
         * @constant
         */
        RECEIVED : 3,

        /**
         * Request was cancelled, either explicitly or automatically because of low priority.
         *
         * @type Number
         * @constant
         */
        CANCELLED : 4,

        /**
         * Request failed.
         *
         * @type Number
         * @constant
         */
        FAILED : 5
    };

    return freezeObject(RequestState);
});

define('Core/RequestType',[
        '../Core/freezeObject'
    ], function(
        freezeObject) {
    'use strict';

    /**
     * An enum identifying the type of request. Used for finer grained logging and priority sorting.
     *
     * @exports RequestType
     */
    var RequestType = {
        /**
         * Terrain request.
         *
         * @type Number
         * @constant
         */
        TERRAIN : 0,

        /**
         * Imagery request.
         *
         * @type Number
         * @constant
         */
        IMAGERY : 1,

        /**
         * 3D Tiles request.
         *
         * @type Number
         * @constant
         */
        TILES3D : 2,

        /**
         * Other request.
         *
         * @type Number
         * @constant
         */
        OTHER : 3
    };

    return freezeObject(RequestType);
});

define('Core/Request',[
        './defaultValue',
        './defined',
        './RequestState',
        './RequestType'
    ], function(
        defaultValue,
        defined,
        RequestState,
        RequestType) {
    'use strict';

    /**
     * Stores information for making a request. In general this does not need to be constructed directly.
     *
     * @alias Request
     * @constructor
     * @namespace
     * @exports Request
     * @param {Object} [options] An object with the following properties:
     * @param {String} [options.url] The url to request.
     * @param {Request~RequestCallback} [options.requestFunction] The function that makes the actual data request.
     * @param {Request~CancelCallback} [options.cancelFunction] The function that is called when the request is cancelled.
     * @param {Request~PriorityCallback} [options.priorityFunction] The function that is called to update the request's priority, which occurs once per frame.
     * @param {Number} [options.priority=0.0] The initial priority of the request.
     * @param {Boolean} [options.throttle=false] Whether to throttle and prioritize the request. If false, the request will be sent immediately. If true, the request will be throttled and sent based on priority.
     * @param {Boolean} [options.throttleByServer=false] Whether to throttle the request by server.
     * @param {RequestType} [options.type=RequestType.OTHER] The type of request.
     */
    function Request(options) {
        options = defaultValue(options, defaultValue.EMPTY_OBJECT);

        var throttleByServer = defaultValue(options.throttleByServer, false);
        var throttle = defaultValue(options.throttle, false);

        /**
         * The URL to request.
         *
         * @type {String}
         */
        this.url = options.url;

        /**
         * The function that makes the actual data request.
         *
         * @type {Request~RequestCallback}
         */
        this.requestFunction = options.requestFunction;

        /**
         * The function that is called when the request is cancelled.
         *
         * @type {Request~CancelCallback}
         */
        this.cancelFunction = options.cancelFunction;

        /**
         * The function that is called to update the request's priority, which occurs once per frame.
         *
         * @type {Request~PriorityCallback}
         */
        this.priorityFunction = options.priorityFunction;

        /**
         * Priority is a unit-less value where lower values represent higher priority.
         * For world-based objects, this is usually the distance from the camera.
         * A request that does not have a priority function defaults to a priority of 0.
         *
         * If priorityFunction is defined, this value is updated every frame with the result of that call.
         *
         * @type {Number}
         * @default 0.0
         */
        this.priority = defaultValue(options.priority, 0.0);

        /**
         * Whether to throttle and prioritize the request. If false, the request will be sent immediately. If true, the
         * request will be throttled and sent based on priority.
         *
         * @type {Boolean}
         * @readonly
         *
         * @default false
         */
        this.throttle = throttle;

        /**
         * Whether to throttle the request by server. Browsers typically support about 6-8 parallel connections
         * for HTTP/1 servers, and an unlimited amount of connections for HTTP/2 servers. Setting this value
         * to <code>true</code> is preferable for requests going through HTTP/1 servers.
         *
         * @type {Boolean}
         * @readonly
         *
         * @default false
         */
        this.throttleByServer = throttleByServer;

        /**
         * Type of request.
         *
         * @type {RequestType}
         * @readonly
         *
         * @default RequestType.OTHER
         */
        this.type = defaultValue(options.type, RequestType.OTHER);

        /**
         * A key used to identify the server that a request is going to. It is derived from the url's authority and scheme.
         *
         * @type {String}
         *
         * @private
         */
        this.serverKey = undefined;

        /**
         * The current state of the request.
         *
         * @type {RequestState}
         * @readonly
         */
        this.state = RequestState.UNISSUED;

        /**
         * The requests's deferred promise.
         *
         * @type {Object}
         *
         * @private
         */
        this.deferred = undefined;

        /**
         * Whether the request was explicitly cancelled.
         *
         * @type {Boolean}
         *
         * @private
         */
        this.cancelled = false;
    }

    /**
     * Mark the request as cancelled.
     *
     * @private
     */
    Request.prototype.cancel = function() {
        this.cancelled = true;
    };

    /**
     * Duplicates a Request instance.
     *
     * @param {Request} [result] The object onto which to store the result.
     *
     * @returns {Request} The modified result parameter or a new Resource instance if one was not provided.
     */
    Request.prototype.clone = function(result) {
        if (!defined(result)) {
            return new Request(this);
        }

        result.url = this.url;
        result.requestFunction = this.requestFunction;
        result.cancelFunction = this.cancelFunction;
        result.priorityFunction = this.priorityFunction;
        result.priority = this.priority;
        result.throttle = this.throttle;
        result.throttleByServer = this.throttleByServer;
        result.type = this.type;
        result.serverKey = this.serverKey;

        // These get defaulted because the cloned request hasn't been issued
        result.state = this.RequestState.UNISSUED;
        result.deferred = undefined;
        result.cancelled = false;

        return result;
    };

    /**
     * The function that makes the actual data request.
     * @callback Request~RequestCallback
     * @returns {Promise} A promise for the requested data.
     */

    /**
     * The function that is called when the request is cancelled.
     * @callback Request~CancelCallback
     */

    /**
     * The function that is called to update the request's priority, which occurs once per frame.
     * @callback Request~PriorityCallback
     * @returns {Number} The updated priority value.
     */

    return Request;
});

define('Core/parseResponseHeaders',[], function() {
    'use strict';

    /**
     * Parses the result of XMLHttpRequest's getAllResponseHeaders() method into
     * a dictionary.
     *
     * @exports parseResponseHeaders
     *
     * @param {String} headerString The header string returned by getAllResponseHeaders().  The format is
     *                 described here: http://www.w3.org/TR/XMLHttpRequest/#the-getallresponseheaders()-method
     * @returns {Object} A dictionary of key/value pairs, where each key is the name of a header and the corresponding value
     *                   is that header's value.
     *
     * @private
     */
    function parseResponseHeaders(headerString) {
        var headers = {};

        if (!headerString) {
          return headers;
        }

        var headerPairs = headerString.split('\u000d\u000a');

        for (var i = 0; i < headerPairs.length; ++i) {
          var headerPair = headerPairs[i];
          // Can't use split() here because it does the wrong thing
          // if the header value has the string ": " in it.
          var index = headerPair.indexOf('\u003a\u0020');
          if (index > 0) {
            var key = headerPair.substring(0, index);
            var val = headerPair.substring(index + 2);
            headers[key] = val;
          }
        }

        return headers;
    }

    return parseResponseHeaders;
});

define('Core/RequestErrorEvent',[
        './defined',
        './parseResponseHeaders'
    ], function(
        defined,
        parseResponseHeaders) {
    'use strict';

    /**
     * An event that is raised when a request encounters an error.
     *
     * @constructor
     * @alias RequestErrorEvent
     *
     * @param {Number} [statusCode] The HTTP error status code, such as 404.
     * @param {Object} [response] The response included along with the error.
     * @param {String|Object} [responseHeaders] The response headers, represented either as an object literal or as a
     *                        string in the format returned by XMLHttpRequest's getAllResponseHeaders() function.
     */
    function RequestErrorEvent(statusCode, response, responseHeaders) {
        /**
         * The HTTP error status code, such as 404.  If the error does not have a particular
         * HTTP code, this property will be undefined.
         *
         * @type {Number}
         */
        this.statusCode = statusCode;

        /**
         * The response included along with the error.  If the error does not include a response,
         * this property will be undefined.
         *
         * @type {Object}
         */
        this.response = response;

        /**
         * The headers included in the response, represented as an object literal of key/value pairs.
         * If the error does not include any headers, this property will be undefined.
         *
         * @type {Object}
         */
        this.responseHeaders = responseHeaders;

        if (typeof this.responseHeaders === 'string') {
            this.responseHeaders = parseResponseHeaders(this.responseHeaders);
        }
    }

    /**
     * Creates a string representing this RequestErrorEvent.
     * @memberof RequestErrorEvent
     *
     * @returns {String} A string representing the provided RequestErrorEvent.
     */
    RequestErrorEvent.prototype.toString = function() {
        var str = 'Request has failed.';
        if (defined(this.statusCode)) {
            str += ' Status Code: ' + this.statusCode;
        }
        return str;
    };

    return RequestErrorEvent;
});

define('Core/Event',[
        './Check',
        './defined',
        './defineProperties'
    ], function(
        Check,
        defined,
        defineProperties) {
    'use strict';

    /**
     * A generic utility class for managing subscribers for a particular event.
     * This class is usually instantiated inside of a container class and
     * exposed as a property for others to subscribe to.
     *
     * @alias Event
     * @constructor
     * @example
     * MyObject.prototype.myListener = function(arg1, arg2) {
     *     this.myArg1Copy = arg1;
     *     this.myArg2Copy = arg2;
     * }
     *
     * var myObjectInstance = new MyObject();
     * var evt = new Cesium.Event();
     * evt.addEventListener(MyObject.prototype.myListener, myObjectInstance);
     * evt.raiseEvent('1', '2');
     * evt.removeEventListener(MyObject.prototype.myListener);
     */
    function Event() {
        this._listeners = [];
        this._scopes = [];
        this._toRemove = [];
        this._insideRaiseEvent = false;
    }

    defineProperties(Event.prototype, {
        /**
         * The number of listeners currently subscribed to the event.
         * @memberof Event.prototype
         * @type {Number}
         * @readonly
         */
        numberOfListeners : {
            get : function() {
                return this._listeners.length - this._toRemove.length;
            }
        }
    });

    /**
     * Registers a callback function to be executed whenever the event is raised.
     * An optional scope can be provided to serve as the <code>this</code> pointer
     * in which the function will execute.
     *
     * @param {Function} listener The function to be executed when the event is raised.
     * @param {Object} [scope] An optional object scope to serve as the <code>this</code>
     *        pointer in which the listener function will execute.
     * @returns {Event~RemoveCallback} A function that will remove this event listener when invoked.
     *
     * @see Event#raiseEvent
     * @see Event#removeEventListener
     */
    Event.prototype.addEventListener = function(listener, scope) {
                Check.typeOf.func('listener', listener);
        
        this._listeners.push(listener);
        this._scopes.push(scope);

        var event = this;
        return function() {
            event.removeEventListener(listener, scope);
        };
    };

    /**
     * Unregisters a previously registered callback.
     *
     * @param {Function} listener The function to be unregistered.
     * @param {Object} [scope] The scope that was originally passed to addEventListener.
     * @returns {Boolean} <code>true</code> if the listener was removed; <code>false</code> if the listener and scope are not registered with the event.
     *
     * @see Event#addEventListener
     * @see Event#raiseEvent
     */
    Event.prototype.removeEventListener = function(listener, scope) {
                Check.typeOf.func('listener', listener);
        
        var listeners = this._listeners;
        var scopes = this._scopes;

        var index = -1;
        for (var i = 0; i < listeners.length; i++) {
            if (listeners[i] === listener && scopes[i] === scope) {
                index = i;
                break;
            }
        }

        if (index !== -1) {
            if (this._insideRaiseEvent) {
                //In order to allow removing an event subscription from within
                //a callback, we don't actually remove the items here.  Instead
                //remember the index they are at and undefined their value.
                this._toRemove.push(index);
                listeners[index] = undefined;
                scopes[index] = undefined;
            } else {
                listeners.splice(index, 1);
                scopes.splice(index, 1);
            }
            return true;
        }

        return false;
    };

    function compareNumber(a,b) {
        return b - a;
    }

    /**
     * Raises the event by calling each registered listener with all supplied arguments.
     *
     * @param {*} arguments This method takes any number of parameters and passes them through to the listener functions.
     *
     * @see Event#addEventListener
     * @see Event#removeEventListener
     */
    Event.prototype.raiseEvent = function() {
        this._insideRaiseEvent = true;

        var i;
        var listeners = this._listeners;
        var scopes = this._scopes;
        var length = listeners.length;

        for (i = 0; i < length; i++) {
            var listener = listeners[i];
            if (defined(listener)) {
                listeners[i].apply(scopes[i], arguments);
            }
        }

        //Actually remove items removed in removeEventListener.
        var toRemove = this._toRemove;
        length = toRemove.length;
        if (length > 0) {
            toRemove.sort(compareNumber);
            for (i = 0; i < length; i++) {
                var index = toRemove[i];
                listeners.splice(index, 1);
                scopes.splice(index, 1);
            }
            toRemove.length = 0;
        }

        this._insideRaiseEvent = false;
    };

    /**
     * A function that removes a listener.
     * @callback Event~RemoveCallback
     */

    return Event;
});

define('Core/Heap',[
        './Check',
        './defaultValue',
        './defined',
        './defineProperties'
    ], function(
        Check,
        defaultValue,
        defined,
        defineProperties) {
    'use strict';

    /**
     * Array implementation of a heap.
     *
     * @alias Heap
     * @constructor
     * @private
     *
     * @param {Object} options Object with the following properties:
     * @param {Heap~ComparatorCallback} options.comparator The comparator to use for the heap. If comparator(a, b) is less than 0, sort a to a lower index than b, otherwise sort to a higher index.
     */
    function Heap(options) {
                Check.typeOf.object('options', options);
        Check.defined('options.comparator', options.comparator);
        
        this._comparator = options.comparator;
        this._array = [];
        this._length = 0;
        this._maximumLength = undefined;
    }

    defineProperties(Heap.prototype, {
        /**
         * Gets the length of the heap.
         *
         * @memberof Heap.prototype
         *
         * @type {Number}
         * @readonly
         */
        length : {
            get : function() {
                return this._length;
            }
        },

        /**
         * Gets the internal array.
         *
         * @memberof Heap.prototype
         *
         * @type {Array}
         * @readonly
         */
        internalArray : {
            get : function() {
                return this._array;
            }
        },

        /**
         * Gets and sets the maximum length of the heap.
         *
         * @memberof Heap.prototype
         *
         * @type {Number}
         */
        maximumLength : {
            get : function() {
                return this._maximumLength;
            },
            set : function(value) {
                this._maximumLength = value;
                if (this._length > value && value > 0) {
                    this._length = value;
                    this._array.length = value;
                }
            }
        },

        /**
         * The comparator to use for the heap. If comparator(a, b) is less than 0, sort a to a lower index than b, otherwise sort to a higher index.
         *
         * @memberof Heap.prototype
         *
         * @type {Heap~ComparatorCallback}
         */
        comparator : {
            get : function() {
                return this._comparator;
            }
        }
    });

    function swap(array, a, b) {
        var temp = array[a];
        array[a] = array[b];
        array[b] = temp;
    }

    /**
     * Resizes the internal array of the heap.
     *
     * @param {Number} [length] The length to resize internal array to. Defaults to the current length of the heap.
     */
    Heap.prototype.reserve = function(length) {
        length = defaultValue(length, this._length);
        this._array.length = length;
    };

    /**
     * Update the heap so that index and all descendants satisfy the heap property.
     *
     * @param {Number} [index=0] The starting index to heapify from.
     */
    Heap.prototype.heapify = function(index) {
        index = defaultValue(index, 0);
        var length = this._length;
        var comparator = this._comparator;
        var array = this._array;
        var candidate = -1;
        var inserting = true;

        while (inserting) {
            var right = 2 * (index + 1);
            var left = right - 1;

            if (left < length && comparator(array[left], array[index]) < 0) {
                candidate = left;
            } else {
                candidate = index;
            }

            if (right < length && comparator(array[right], array[candidate]) < 0) {
                candidate = right;
            }
            if (candidate !== index) {
                swap(array, candidate, index);
                index = candidate;
            } else {
                inserting = false;
            }
        }
    };

    /**
     * Resort the heap.
     */
    Heap.prototype.resort = function() {
        var length = this._length;
        for (var i = Math.ceil(length / 2); i >= 0; --i) {
            this.heapify(i);
        }
    };

    /**
     * Insert an element into the heap. If the length would grow greater than maximumLength
     * of the heap, extra elements are removed.
     *
     * @param {*} element The element to insert
     *
     * @return {*} The element that was removed from the heap if the heap is at full capacity.
     */
    Heap.prototype.insert = function(element) {
                Check.defined('element', element);
        
        var array = this._array;
        var comparator = this._comparator;
        var maximumLength = this._maximumLength;

        var index = this._length++;
        if (index < array.length) {
            array[index] = element;
        } else {
            array.push(element);
        }

        while (index !== 0) {
            var parent = Math.floor((index - 1) / 2);
            if (comparator(array[index], array[parent]) < 0) {
                swap(array, index, parent);
                index = parent;
            } else {
                break;
            }
        }

        var removedElement;

        if (defined(maximumLength) && (this._length > maximumLength)) {
            removedElement = array[maximumLength];
            this._length = maximumLength;
        }

        return removedElement;
    };

    /**
     * Remove the element specified by index from the heap and return it.
     *
     * @param {Number} [index=0] The index to remove.
     * @returns {*} The specified element of the heap.
     */
    Heap.prototype.pop = function(index) {
        index = defaultValue(index, 0);
        if (this._length === 0) {
            return undefined;
        }
                Check.typeOf.number.lessThan('index', index, this._length);
        
        var array = this._array;
        var root = array[index];
        swap(array, index, --this._length);
        this.heapify(index);
        return root;
    };

    /**
     * The comparator to use for the heap.
     * @callback Heap~ComparatorCallback
     * @param {*} a An element in the heap.
     * @param {*} b An element in the heap.
     * @returns {Number} If the result of the comparison is less than 0, sort a to a lower index than b, otherwise sort to a higher index.
     */

    return Heap;
});

define('Core/RequestScheduler',[
        '../ThirdParty/Uri',
        '../ThirdParty/when',
        './Check',
        './defaultValue',
        './defined',
        './defineProperties',
        './Event',
        './Heap',
        './isBlobUri',
        './isDataUri',
        './RequestState'
    ], function(
        Uri,
        when,
        Check,
        defaultValue,
        defined,
        defineProperties,
        Event,
        Heap,
        isBlobUri,
        isDataUri,
        RequestState) {
    'use strict';

    function sortRequests(a, b) {
        return a.priority - b.priority;
    }

    var statistics = {
        numberOfAttemptedRequests : 0,
        numberOfActiveRequests : 0,
        numberOfCancelledRequests : 0,
        numberOfCancelledActiveRequests : 0,
        numberOfFailedRequests : 0,
        numberOfActiveRequestsEver : 0,
        lastNumberOfActiveRequests : 0
    };

    var priorityHeapLength = 20;
    var requestHeap = new Heap({
        comparator : sortRequests
    });
    requestHeap.maximumLength = priorityHeapLength;
    requestHeap.reserve(priorityHeapLength);

    var activeRequests = [];
    var numberOfActiveRequestsByServer = {};

    var pageUri = typeof document !== 'undefined' ? new Uri(document.location.href) : new Uri();

    var requestCompletedEvent = new Event();

    /**
     * Tracks the number of active requests and prioritizes incoming requests.
     *
     * @exports RequestScheduler
     *
     * @private
     */
    function RequestScheduler() {
    }

    /**
     * The maximum number of simultaneous active requests. Un-throttled requests do not observe this limit.
     * @type {Number}
     * @default 50
     */
    RequestScheduler.maximumRequests = 50;

    /**
     * The maximum number of simultaneous active requests per server. Un-throttled requests or servers specifically
     * listed in requestsByServer do not observe this limit.
     * @type {Number}
     * @default 6
     */
    RequestScheduler.maximumRequestsPerServer = 6;

    /**
     * A per serverKey list of overrides to use for throttling instead of maximumRequestsPerServer
     */
    RequestScheduler.requestsByServer = {
        'api.cesium.com:443': 18,
        'assets.cesium.com:443': 18
    };

    /**
     * Specifies if the request scheduler should throttle incoming requests, or let the browser queue requests under its control.
     * @type {Boolean}
     * @default true
     */
    RequestScheduler.throttleRequests = true;

    /**
     * When true, log statistics to the console every frame
     * @type {Boolean}
     * @default false
     */
    RequestScheduler.debugShowStatistics = false;

    /**
     * An event that's raised when a request is completed.  Event handlers are passed
     * the error object if the request fails.
     *
     * @type {Event}
     * @default Event()
     */
    RequestScheduler.requestCompletedEvent = requestCompletedEvent;

    defineProperties(RequestScheduler, {
        /**
         * Returns the statistics used by the request scheduler.
         *
         * @memberof RequestScheduler
         *
         * @type Object
         * @readonly
         */
        statistics : {
            get : function() {
                return statistics;
            }
        },

        /**
         * The maximum size of the priority heap. This limits the number of requests that are sorted by priority. Only applies to requests that are not yet active.
         *
         * @memberof RequestScheduler
         *
         * @type {Number}
         * @default 20
         */
        priorityHeapLength : {
            get : function() {
                return priorityHeapLength;
            },
            set : function(value) {
                // If the new length shrinks the heap, need to cancel some of the requests.
                // Since this value is not intended to be tweaked regularly it is fine to just cancel the high priority requests.
                if (value < priorityHeapLength) {
                    while (requestHeap.length > value) {
                        var request = requestHeap.pop();
                        cancelRequest(request);
                    }
                }
                priorityHeapLength = value;
                requestHeap.maximumLength = value;
                requestHeap.reserve(value);
            }
        }
    });

    function updatePriority(request) {
        if (defined(request.priorityFunction)) {
            request.priority = request.priorityFunction();
        }
    }

    function serverHasOpenSlots(serverKey) {
        var maxRequests = defaultValue(RequestScheduler.requestsByServer[serverKey], RequestScheduler.maximumRequestsPerServer);
        return numberOfActiveRequestsByServer[serverKey] < maxRequests;
    }

    function issueRequest(request) {
        if (request.state === RequestState.UNISSUED) {
            request.state = RequestState.ISSUED;
            request.deferred = when.defer();
        }
        return request.deferred.promise;
    }

    function getRequestReceivedFunction(request) {
        return function(results) {
            if (request.state === RequestState.CANCELLED) {
                // If the data request comes back but the request is cancelled, ignore it.
                return;
            }
            --statistics.numberOfActiveRequests;
            --numberOfActiveRequestsByServer[request.serverKey];
            requestCompletedEvent.raiseEvent();
            request.state = RequestState.RECEIVED;
            request.deferred.resolve(results);
        };
    }

    function getRequestFailedFunction(request) {
        return function(error) {
            if (request.state === RequestState.CANCELLED) {
                // If the data request comes back but the request is cancelled, ignore it.
                return;
            }
            ++statistics.numberOfFailedRequests;
            --statistics.numberOfActiveRequests;
            --numberOfActiveRequestsByServer[request.serverKey];
            requestCompletedEvent.raiseEvent(error);
            request.state = RequestState.FAILED;
            request.deferred.reject(error);
        };
    }

    function startRequest(request) {
        var promise = issueRequest(request);
        request.state = RequestState.ACTIVE;
        activeRequests.push(request);
        ++statistics.numberOfActiveRequests;
        ++statistics.numberOfActiveRequestsEver;
        ++numberOfActiveRequestsByServer[request.serverKey];
        request.requestFunction().then(getRequestReceivedFunction(request)).otherwise(getRequestFailedFunction(request));
        return promise;
    }

    function cancelRequest(request) {
        var active = request.state === RequestState.ACTIVE;
        request.state = RequestState.CANCELLED;
        ++statistics.numberOfCancelledRequests;
        request.deferred.reject();

        if (active) {
            --statistics.numberOfActiveRequests;
            --numberOfActiveRequestsByServer[request.serverKey];
            ++statistics.numberOfCancelledActiveRequests;
        }

        if (defined(request.cancelFunction)) {
            request.cancelFunction();
        }
    }

    /**
     * Sort requests by priority and start requests.
     */
    RequestScheduler.update = function() {
        var i;
        var request;

        // Loop over all active requests. Cancelled, failed, or received requests are removed from the array to make room for new requests.
        var removeCount = 0;
        var activeLength = activeRequests.length;
        for (i = 0; i < activeLength; ++i) {
            request = activeRequests[i];
            if (request.cancelled) {
                // Request was explicitly cancelled
                cancelRequest(request);
            }
            if (request.state !== RequestState.ACTIVE) {
                // Request is no longer active, remove from array
                ++removeCount;
                continue;
            }
            if (removeCount > 0) {
                // Shift back to fill in vacated slots from completed requests
                activeRequests[i - removeCount] = request;
            }
        }
        activeRequests.length -= removeCount;

        // Update priority of issued requests and resort the heap
        var issuedRequests = requestHeap.internalArray;
        var issuedLength = requestHeap.length;
        for (i = 0; i < issuedLength; ++i) {
            updatePriority(issuedRequests[i]);
        }
        requestHeap.resort();

        // Get the number of open slots and fill with the highest priority requests.
        // Un-throttled requests are automatically added to activeRequests, so activeRequests.length may exceed maximumRequests
        var openSlots = Math.max(RequestScheduler.maximumRequests - activeRequests.length, 0);
        var filledSlots = 0;
        while (filledSlots < openSlots && requestHeap.length > 0) {
            // Loop until all open slots are filled or the heap becomes empty
            request = requestHeap.pop();
            if (request.cancelled) {
                // Request was explicitly cancelled
                cancelRequest(request);
                continue;
            }

            if (request.throttleByServer && !serverHasOpenSlots(request.serverKey)) {
                // Open slots are available, but the request is throttled by its server. Cancel and try again later.
                cancelRequest(request);
                continue;
            }

            startRequest(request);
            ++filledSlots;
        }

        updateStatistics();
    };

    /**
     * Get the server key from a given url.
     *
     * @param {String} url The url.
     * @returns {String} The server key.
     */
    RequestScheduler.getServerKey = function(url) {
                Check.typeOf.string('url', url);
        
        var uri = new Uri(url).resolve(pageUri);
        uri.normalize();
        var serverKey = uri.authority;
        if (!/:/.test(serverKey)) {
            // If the authority does not contain a port number, add port 443 for https or port 80 for http
            serverKey = serverKey + ':' + (uri.scheme === 'https' ? '443' : '80');
        }

        var length = numberOfActiveRequestsByServer[serverKey];
        if (!defined(length)) {
            numberOfActiveRequestsByServer[serverKey] = 0;
        }

        return serverKey;
    };

    /**
     * Issue a request. If request.throttle is false, the request is sent immediately. Otherwise the request will be
     * queued and sorted by priority before being sent.
     *
     * @param {Request} request The request object.
     *
     * @returns {Promise|undefined} A Promise for the requested data, or undefined if this request does not have high enough priority to be issued.
     */
    RequestScheduler.request = function(request) {
                Check.typeOf.object('request', request);
        Check.typeOf.string('request.url', request.url);
        Check.typeOf.func('request.requestFunction', request.requestFunction);
        
        if (isDataUri(request.url) || isBlobUri(request.url)) {
            requestCompletedEvent.raiseEvent();
            request.state = RequestState.RECEIVED;
            return request.requestFunction();
        }

        ++statistics.numberOfAttemptedRequests;

        if (!defined(request.serverKey)) {
            request.serverKey = RequestScheduler.getServerKey(request.url);
        }

        if (request.throttleByServer && !serverHasOpenSlots(request.serverKey)) {
            // Server is saturated. Try again later.
            return undefined;
        }

        if (!RequestScheduler.throttleRequests || !request.throttle) {
            return startRequest(request);
        }

        if (activeRequests.length >= RequestScheduler.maximumRequests) {
            // Active requests are saturated. Try again later.
            return undefined;
        }

        // Insert into the priority heap and see if a request was bumped off. If this request is the lowest
        // priority it will be returned.
        updatePriority(request);
        var removedRequest = requestHeap.insert(request);

        if (defined(removedRequest)) {
            if (removedRequest === request) {
                // Request does not have high enough priority to be issued
                return undefined;
            }
            // A previously issued request has been bumped off the priority heap, so cancel it
            cancelRequest(removedRequest);
        }

        return issueRequest(request);
    };

    function updateStatistics() {
        if (!RequestScheduler.debugShowStatistics) {
            return;
        }

        if (statistics.numberOfActiveRequests === 0 && statistics.lastNumberOfActiveRequests > 0) {
            if (statistics.numberOfAttemptedRequests > 0) {
                console.log('Number of attempted requests: ' + statistics.numberOfAttemptedRequests);
                statistics.numberOfAttemptedRequests = 0;
            }

            if (statistics.numberOfCancelledRequests > 0) {
                console.log('Number of cancelled requests: ' + statistics.numberOfCancelledRequests);
                statistics.numberOfCancelledRequests = 0;
            }

            if (statistics.numberOfCancelledActiveRequests > 0) {
                console.log('Number of cancelled active requests: ' + statistics.numberOfCancelledActiveRequests);
                statistics.numberOfCancelledActiveRequests = 0;
            }

            if (statistics.numberOfFailedRequests > 0) {
                console.log('Number of failed requests: ' + statistics.numberOfFailedRequests);
                statistics.numberOfFailedRequests = 0;
            }
        }

        statistics.lastNumberOfActiveRequests = statistics.numberOfActiveRequests;
    }

    /**
     * For testing only. Clears any requests that may not have completed from previous tests.
     *
     * @private
     */
    RequestScheduler.clearForSpecs = function() {
        while (requestHeap.length > 0) {
            var request = requestHeap.pop();
            cancelRequest(request);
        }
        var length = activeRequests.length;
        for (var i = 0; i < length; ++i) {
            cancelRequest(activeRequests[i]);
        }
        activeRequests.length = 0;
        numberOfActiveRequestsByServer = {};

        // Clear stats
        statistics.numberOfAttemptedRequests = 0;
        statistics.numberOfActiveRequests = 0;
        statistics.numberOfCancelledRequests = 0;
        statistics.numberOfCancelledActiveRequests = 0;
        statistics.numberOfFailedRequests = 0;
        statistics.numberOfActiveRequestsEver = 0;
        statistics.lastNumberOfActiveRequests = 0;
    };

    /**
     * For testing only.
     *
     * @private
     */
    RequestScheduler.numberOfActiveRequestsByServer = function(serverKey) {
        return numberOfActiveRequestsByServer[serverKey];
    };

    /**
     * For testing only.
     *
     * @private
     */
    RequestScheduler.requestHeap = requestHeap;

    return RequestScheduler;
});

define('Core/TrustedServers',[
        '../ThirdParty/Uri',
        './defined',
        './DeveloperError'
    ], function(
        Uri,
        defined,
        DeveloperError) {
    'use strict';

    /**
     * A singleton that contains all of the servers that are trusted. Credentials will be sent with
     * any requests to these servers.
     *
     * @exports TrustedServers
     *
     * @see {@link http://www.w3.org/TR/cors/|Cross-Origin Resource Sharing}
     */
    var TrustedServers = {};
    var _servers = {};

    /**
     * Adds a trusted server to the registry
     *
     * @param {String} host The host to be added.
     * @param {Number} port The port used to access the host.
     *
     * @example
     * // Add a trusted server
     * TrustedServers.add('my.server.com', 80);
     */
    TrustedServers.add = function(host, port) {
                if (!defined(host)) {
            throw new DeveloperError('host is required.');
        }
        if (!defined(port) || port <= 0) {
            throw new DeveloperError('port is required to be greater than 0.');
        }
        
        var authority = host.toLowerCase() + ':' + port;
        if (!defined(_servers[authority])) {
            _servers[authority] = true;
        }
    };

    /**
     * Removes a trusted server from the registry
     *
     * @param {String} host The host to be removed.
     * @param {Number} port The port used to access the host.
     *
     * @example
     * // Remove a trusted server
     * TrustedServers.remove('my.server.com', 80);
     */
    TrustedServers.remove = function(host, port) {
                if (!defined(host)) {
            throw new DeveloperError('host is required.');
        }
        if (!defined(port) || port <= 0) {
            throw new DeveloperError('port is required to be greater than 0.');
        }
        
        var authority = host.toLowerCase() + ':' + port;
        if (defined(_servers[authority])) {
            delete _servers[authority];
        }
    };

    function getAuthority(url) {
        var uri = new Uri(url);
        uri.normalize();

        // Removes username:password@ so we just have host[:port]
        var authority = uri.getAuthority();
        if (!defined(authority)) {
            return undefined; // Relative URL
        }

        if (authority.indexOf('@') !== -1) {
            var parts = authority.split('@');
            authority = parts[1];
        }

        // If the port is missing add one based on the scheme
        if (authority.indexOf(':') === -1) {
            var scheme = uri.getScheme();
            if (!defined(scheme)) {
                scheme = window.location.protocol;
                scheme = scheme.substring(0, scheme.length-1);
            }
            if (scheme === 'http') {
                authority += ':80';
            } else if (scheme === 'https') {
                authority += ':443';
            } else {
                return undefined;
            }
        }

        return authority;
    }

    /**
     * Tests whether a server is trusted or not. The server must have been added with the port if it is included in the url.
     *
     * @param {String} url The url to be tested against the trusted list
     *
     * @returns {boolean} Returns true if url is trusted, false otherwise.
     *
     * @example
     * // Add server
     * TrustedServers.add('my.server.com', 81);
     *
     * // Check if server is trusted
     * if (TrustedServers.contains('https://my.server.com:81/path/to/file.png')) {
     *     // my.server.com:81 is trusted
     * }
     * if (TrustedServers.contains('https://my.server.com/path/to/file.png')) {
     *     // my.server.com isn't trusted
     * }
     */
    TrustedServers.contains = function(url) {
                if (!defined(url)) {
            throw new DeveloperError('url is required.');
        }
                var authority = getAuthority(url);
        if (defined(authority) && defined(_servers[authority])) {
            return true;
        }

        return false;
    };

    /**
     * Clears the registry
     *
     * @example
     * // Remove a trusted server
     * TrustedServers.clear();
     */
    TrustedServers.clear = function() {
        _servers = {};
    };

    return TrustedServers;
});

define('Core/Resource',[
        '../ThirdParty/Uri',
        '../ThirdParty/when',
        './appendForwardSlash',
        './Check',
        './clone',
        './combine',
        './defaultValue',
        './defined',
        './defineProperties',
        './DeveloperError',
        './freezeObject',
        './getAbsoluteUri',
        './getBaseUri',
        './getExtensionFromUri',
        './isBlobUri',
        './isCrossOriginUrl',
        './isDataUri',
        './loadAndExecuteScript',
        './objectToQuery',
        './queryToObject',
        './Request',
        './RequestErrorEvent',
        './RequestScheduler',
        './RequestState',
        './RuntimeError',
        './TrustedServers'
    ], function(
        Uri,
        when,
        appendForwardSlash,
        Check,
        clone,
        combine,
        defaultValue,
        defined,
        defineProperties,
        DeveloperError,
        freezeObject,
        getAbsoluteUri,
        getBaseUri,
        getExtensionFromUri,
        isBlobUri,
        isCrossOriginUrl,
        isDataUri,
        loadAndExecuteScript,
        objectToQuery,
        queryToObject,
        Request,
        RequestErrorEvent,
        RequestScheduler,
        RequestState,
        RuntimeError,
        TrustedServers) {
    'use strict';

    var xhrBlobSupported = (function() {
        try {
            var xhr = new XMLHttpRequest();
            xhr.open('GET', '#', true);
            xhr.responseType = 'blob';
            return xhr.responseType === 'blob';
        } catch (e) {
            return false;
        }
    })();

    /**
     * Parses a query string and returns the object equivalent.
     *
     * @param {Uri} uri The Uri with a query object.
     * @param {Resource} resource The Resource that will be assigned queryParameters.
     * @param {Boolean} merge If true, we'll merge with the resource's existing queryParameters. Otherwise they will be replaced.
     * @param {Boolean} preserveQueryParameters If true duplicate parameters will be concatenated into an array. If false, keys in uri will take precedence.
     *
     * @private
     */
    function parseQuery(uri, resource, merge, preserveQueryParameters) {
        var queryString = uri.query;
        if (!defined(queryString) || (queryString.length === 0)) {
            return {};
        }

        var query;
        // Special case we run into where the querystring is just a string, not key/value pairs
        if (queryString.indexOf('=') === -1) {
            var result = {};
            result[queryString] = undefined;
            query = result;
        } else {
            query = queryToObject(queryString);
        }

        if (merge) {
            resource._queryParameters = combineQueryParameters(query, resource._queryParameters, preserveQueryParameters);
        } else {
            resource._queryParameters = query;
        }
        uri.query = undefined;
    }

    /**
     * Converts a query object into a string.
     *
     * @param {Uri} uri The Uri object that will have the query object set.
     * @param {Resource} resource The resource that has queryParameters
     *
     * @private
     */
    function stringifyQuery(uri, resource) {
        var queryObject = resource._queryParameters;

        var keys = Object.keys(queryObject);

        // We have 1 key with an undefined value, so this is just a string, not key/value pairs
        if (keys.length === 1 && !defined(queryObject[keys[0]])) {
            uri.query = keys[0];
        } else {
            uri.query = objectToQuery(queryObject);
        }
    }

    /**
     * Clones a value if it is defined, otherwise returns the default value
     *
     * @param {*} [val] The value to clone.
     * @param {*} [defaultVal] The default value.
     *
     * @returns {*} A clone of val or the defaultVal.
     *
     * @private
     */
    function defaultClone(val, defaultVal) {
        if (!defined(val)) {
            return defaultVal;
        }

        return defined(val.clone) ? val.clone() : clone(val);
    }

    /**
     * Checks to make sure the Resource isn't already being requested.
     *
     * @param {Request} request The request to check.
     *
     * @private
     */
    function checkAndResetRequest(request) {
        if (request.state === RequestState.ISSUED || request.state === RequestState.ACTIVE) {
            throw new RuntimeError('The Resource is already being fetched.');
        }

        request.state = RequestState.UNISSUED;
        request.deferred = undefined;
    }

    /**
     * This combines a map of query parameters.
     *
     * @param {Object} q1 The first map of query parameters. Values in this map will take precedence if preserveQueryParameters is false.
     * @param {Object} q2 The second map of query parameters.
     * @param {Boolean} preserveQueryParameters If true duplicate parameters will be concatenated into an array. If false, keys in q1 will take precedence.
     *
     * @returns {Object} The combined map of query parameters.
     *
     * @example
     * var q1 = {
     *   a: 1,
     *   b: 2
     * };
     * var q2 = {
     *   a: 3,
     *   c: 4
     * };
     * var q3 = {
     *   b: [5, 6],
     *   d: 7
     * }
     *
     * // Returns
     * // {
     * //   a: [1, 3],
     * //   b: 2,
     * //   c: 4
     * // };
     * combineQueryParameters(q1, q2, true);
     *
     * // Returns
     * // {
     * //   a: 1,
     * //   b: 2,
     * //   c: 4
     * // };
     * combineQueryParameters(q1, q2, false);
     *
     * // Returns
     * // {
     * //   a: 1,
     * //   b: [2, 5, 6],
     * //   d: 7
     * // };
     * combineQueryParameters(q1, q3, true);
     *
     * // Returns
     * // {
     * //   a: 1,
     * //   b: 2,
     * //   d: 7
     * // };
     * combineQueryParameters(q1, q3, false);
     *
     * @private
     */
    function combineQueryParameters(q1, q2, preserveQueryParameters) {
        if (!preserveQueryParameters) {
            return combine(q1, q2);
        }

        var result = clone(q1, true);
        for (var param in q2) {
            if (q2.hasOwnProperty(param)) {
                var value = result[param];
                var q2Value = q2[param];
                if (defined(value)) {
                    if (!Array.isArray(value)) {
                        value = result[param] = [value];
                    }

                    result[param] = value.concat(q2Value);
                } else {
                    result[param] = Array.isArray(q2Value) ? q2Value.slice() : q2Value;
                }
            }
        }

        return result;
    }

    /**
     * A resource that includes the location and any other parameters we need to retrieve it or create derived resources. It also provides the ability to retry requests.
     *
     * @alias Resource
     * @constructor
     *
     * @param {String|Object} options A url or an object with the following properties
     * @param {String} options.url The url of the resource.
     * @param {Object} [options.queryParameters] An object containing query parameters that will be sent when retrieving the resource.
     * @param {Object} [options.templateValues] Key/Value pairs that are used to replace template values (eg. {x}).
     * @param {Object} [options.headers={}] Additional HTTP headers that will be sent.
     * @param {DefaultProxy} [options.proxy] A proxy to be used when loading the resource.
     * @param {Resource~RetryCallback} [options.retryCallback] The Function to call when a request for this resource fails. If it returns true, the request will be retried.
     * @param {Number} [options.retryAttempts=0] The number of times the retryCallback should be called before giving up.
     * @param {Request} [options.request] A Request object that will be used. Intended for internal use only.
     *
     * @example
     * function refreshTokenRetryCallback(resource, error) {
     *   if (error.statusCode === 403) {
     *     // 403 status code means a new token should be generated
     *     return getNewAccessToken()
     *       .then(function(token) {
     *         resource.queryParameters.access_token = token;
     *         return true;
     *       })
     *       .otherwise(function() {
     *         return false;
     *       });
     *   }
     *
     *   return false;
     * }
     *
     * var resource = new Resource({
     *    url: 'http://server.com/path/to/resource.json',
     *    proxy: new DefaultProxy('/proxy/'),
     *    headers: {
     *      'X-My-Header': 'valueOfHeader'
     *    },
     *    queryParameters: {
     *      'access_token': '123-435-456-000'
     *    },
     *    retryCallback: refreshTokenRetryCallback,
     *    retryAttempts: 1
     * });
     */
    function Resource(options) {
        options = defaultValue(options, defaultValue.EMPTY_OBJECT);
        if (typeof options === 'string') {
            options = {
                url: options
            };
        }

                Check.typeOf.string('options.url', options.url);
        
        this._url = undefined;
        this._templateValues = defaultClone(options.templateValues, {});
        this._queryParameters = defaultClone(options.queryParameters, {});

        /**
         * Additional HTTP headers that will be sent with the request.
         *
         * @type {Object}
         */
        this.headers = defaultClone(options.headers, {});

        /**
         * A Request object that will be used. Intended for internal use only.
         *
         * @type {Request}
         */
        this.request = defaultValue(options.request, new Request());

        /**
         * A proxy to be used when loading the resource.
         *
         * @type {DefaultProxy}
         */
        this.proxy = options.proxy;

        /**
         * Function to call when a request for this resource fails. If it returns true or a Promise that resolves to true, the request will be retried.
         *
         * @type {Function}
         */
        this.retryCallback = options.retryCallback;

        /**
         * The number of times the retryCallback should be called before giving up.
         *
         * @type {Number}
         */
        this.retryAttempts = defaultValue(options.retryAttempts, 0);
        this._retryCount = 0;

        var uri = new Uri(options.url);
        parseQuery(uri, this, true, true);

        // Remove the fragment as it's not sent with a request
        uri.fragment = undefined;

        this._url = uri.toString();
    }

    /**
     * A helper function to create a resource depending on whether we have a String or a Resource
     *
     * @param {Resource|String} resource A Resource or a String to use when creating a new Resource.
     *
     * @returns {Resource} If resource is a String, a Resource constructed with the url and options. Otherwise the resource parameter is returned.
     *
     * @private
     */
    Resource.createIfNeeded = function(resource) {
        if (resource instanceof Resource) {
            // Keep existing request object. This function is used internally to duplicate a Resource, so that it can't
            //  be modified outside of a class that holds it (eg. an imagery or terrain provider). Since the Request objects
            //  are managed outside of the providers, by the tile loading code, we want to keep the request property the same so if it is changed
            //  in the underlying tiling code the requests for this resource will use it.
            return  resource.getDerivedResource({
                request: resource.request
            });
        }

        if (typeof resource !== 'string') {
            return resource;
        }

        return new Resource({
            url: resource
        });
    };

    var supportsImageBitmapOptionsPromise;
    /**
     * A helper function to check whether createImageBitmap supports passing ImageBitmapOptions.
     *
     * @returns {Promise<Boolean>} A promise that resolves to true if this browser supports creating an ImageBitmap with options.
     *
     * @private
     */
    Resource.supportsImageBitmapOptions = function() {
        // Until the HTML folks figure out what to do about this, we need to actually try loading an image to
        // know if this browser supports passing options to the createImageBitmap function.
        // https://github.com/whatwg/html/pull/4248
        if (defined(supportsImageBitmapOptionsPromise)) {
            return supportsImageBitmapOptionsPromise;
        }

        if (typeof createImageBitmap !== 'function') {
            supportsImageBitmapOptionsPromise = when.resolve(false);
            return supportsImageBitmapOptionsPromise;
        }

        var imageDataUri = 'data:image/png;base64,iVBORw0KGgoAAAANSUhEUgAAAAEAAAABCAYAAAAfFcSJAAAADUlEQVQImWP4////fwAJ+wP9CNHoHgAAAABJRU5ErkJggg==';

        supportsImageBitmapOptionsPromise = Resource.fetchBlob({
            url : imageDataUri
        })
            .then(function(blob) {
                return createImageBitmap(blob, {
                    imageOrientation: 'flipY',
                    premultiplyAlpha: 'none'
                });
            })
            .then(function(imageBitmap) {
                return true;
            })
            .otherwise(function() {
                return false;
            });

        return supportsImageBitmapOptionsPromise;
    };

    defineProperties(Resource, {
        /**
         * Returns true if blobs are supported.
         *
         * @memberof Resource
         * @type {Boolean}
         *
         * @readonly
         */
        isBlobSupported : {
            get : function() {
                return xhrBlobSupported;
            }
        }
    });

    defineProperties(Resource.prototype, {
        /**
         * Query parameters appended to the url.
         *
         * @memberof Resource.prototype
         * @type {Object}
         *
         * @readonly
         */
        queryParameters: {
            get: function() {
                return this._queryParameters;
            }
        },

        /**
         * The key/value pairs used to replace template parameters in the url.
         *
         * @memberof Resource.prototype
         * @type {Object}
         *
         * @readonly
         */
        templateValues: {
            get: function() {
                return this._templateValues;
            }
        },

        /**
         * The url to the resource with template values replaced, query string appended and encoded by proxy if one was set.
         *
         * @memberof Resource.prototype
         * @type {String}
         */
        url: {
            get: function() {
                return this.getUrlComponent(true, true);
            },
            set: function(value) {
                var uri = new Uri(value);

                parseQuery(uri, this, false);

                // Remove the fragment as it's not sent with a request
                uri.fragment = undefined;

                this._url = uri.toString();
            }
        },

        /**
         * The file extension of the resource.
         *
         * @memberof Resource.prototype
         * @type {String}
         *
         * @readonly
         */
        extension: {
            get: function() {
                return getExtensionFromUri(this._url);
            }
        },

        /**
         * True if the Resource refers to a data URI.
         *
         * @memberof Resource.prototype
         * @type {Boolean}
         */
        isDataUri: {
            get: function() {
                return isDataUri(this._url);
            }
        },

        /**
         * True if the Resource refers to a blob URI.
         *
         * @memberof Resource.prototype
         * @type {Boolean}
         */
        isBlobUri: {
            get: function() {
                return isBlobUri(this._url);
            }
        },

        /**
         * True if the Resource refers to a cross origin URL.
         *
         * @memberof Resource.prototype
         * @type {Boolean}
         */
        isCrossOriginUrl: {
            get: function() {
                return isCrossOriginUrl(this._url);
            }
        },

        /**
         * True if the Resource has request headers. This is equivalent to checking if the headers property has any keys.
         *
         * @memberof Resource.prototype
         * @type {Boolean}
         */
        hasHeaders: {
            get: function() {
                return (Object.keys(this.headers).length > 0);
            }
        }
    });

    /**
     * Returns the url, optional with the query string and processed by a proxy.
     *
     * @param {Boolean} [query=false] If true, the query string is included.
     * @param {Boolean} [proxy=false] If true, the url is processed the proxy object if defined.
     *
     * @returns {String} The url with all the requested components.
     */
    Resource.prototype.getUrlComponent = function(query, proxy) {
        if(this.isDataUri) {
            return this._url;
        }

        var uri = new Uri(this._url);

        if (query) {
            stringifyQuery(uri, this);
        }

        // objectToQuery escapes the placeholders.  Undo that.
        var url = uri.toString().replace(/%7B/g, '{').replace(/%7D/g, '}');

        var templateValues = this._templateValues;
        url = url.replace(/{(.*?)}/g, function(match, key) {
            var replacement = templateValues[key];
            if (defined(replacement)) {
                // use the replacement value from templateValues if there is one...
                return encodeURIComponent(replacement);
            }
            // otherwise leave it unchanged
            return match;
        });

        if (proxy && defined(this.proxy)) {
            url = this.proxy.getURL(url);
        }
        return url;
    };

    /**
     * Combines the specified object and the existing query parameters. This allows you to add many parameters at once,
     *  as opposed to adding them one at a time to the queryParameters property. If a value is already set, it will be replaced with the new value.
     *
     * @param {Object} params The query parameters
     * @param {Boolean} [useAsDefault=false] If true the params will be used as the default values, so they will only be set if they are undefined.
     */
    Resource.prototype.setQueryParameters = function(params, useAsDefault) {
        if (useAsDefault) {
            this._queryParameters = combineQueryParameters(this._queryParameters, params, false);
        } else {
            this._queryParameters = combineQueryParameters(params, this._queryParameters, false);
        }
    };

    /**
     * Combines the specified object and the existing query parameters. This allows you to add many parameters at once,
     *  as opposed to adding them one at a time to the queryParameters property.
     *
     * @param {Object} params The query parameters
     */
    Resource.prototype.appendQueryParameters = function(params) {
        this._queryParameters = combineQueryParameters(params, this._queryParameters, true);
    };

    /**
     * Combines the specified object and the existing template values. This allows you to add many values at once,
     *  as opposed to adding them one at a time to the templateValues property. If a value is already set, it will become an array and the new value will be appended.
     *
     * @param {Object} template The template values
     * @param {Boolean} [useAsDefault=false] If true the values will be used as the default values, so they will only be set if they are undefined.
     */
    Resource.prototype.setTemplateValues = function(template, useAsDefault) {
        if (useAsDefault) {
            this._templateValues = combine(this._templateValues, template);
        } else {
            this._templateValues = combine(template, this._templateValues);
        }
    };

    /**
     * Returns a resource relative to the current instance. All properties remain the same as the current instance unless overridden in options.
     *
     * @param {Object} options An object with the following properties
     * @param {String} [options.url]  The url that will be resolved relative to the url of the current instance.
     * @param {Object} [options.queryParameters] An object containing query parameters that will be combined with those of the current instance.
     * @param {Object} [options.templateValues] Key/Value pairs that are used to replace template values (eg. {x}). These will be combined with those of the current instance.
     * @param {Object} [options.headers={}] Additional HTTP headers that will be sent.
     * @param {DefaultProxy} [options.proxy] A proxy to be used when loading the resource.
     * @param {Resource~RetryCallback} [options.retryCallback] The function to call when loading the resource fails.
     * @param {Number} [options.retryAttempts] The number of times the retryCallback should be called before giving up.
     * @param {Request} [options.request] A Request object that will be used. Intended for internal use only.
     * @param {Boolean} [options.preserveQueryParameters=false] If true, this will keep all query parameters from the current resource and derived resource. If false, derived parameters will replace those of the current resource.
     *
     * @returns {Resource} The resource derived from the current one.
     */
    Resource.prototype.getDerivedResource = function(options) {
        var resource = this.clone();
        resource._retryCount = 0;

        if (defined(options.url)) {
            var uri = new Uri(options.url);

            var preserveQueryParameters = defaultValue(options.preserveQueryParameters, false);
            parseQuery(uri, resource, true, preserveQueryParameters);

            // Remove the fragment as it's not sent with a request
            uri.fragment = undefined;

            resource._url = uri.resolve(new Uri(getAbsoluteUri(this._url))).toString();
        }

        if (defined(options.queryParameters)) {
            resource._queryParameters = combine(options.queryParameters, resource._queryParameters);
        }
        if (defined(options.templateValues)) {
            resource._templateValues = combine(options.templateValues, resource.templateValues);
        }
        if (defined(options.headers)) {
            resource.headers = combine(options.headers, resource.headers);
        }
        if (defined(options.proxy)) {
            resource.proxy = options.proxy;
        }
        if (defined(options.request)) {
            resource.request = options.request;
        }
        if (defined(options.retryCallback)) {
            resource.retryCallback = options.retryCallback;
        }
        if (defined(options.retryAttempts)) {
            resource.retryAttempts = options.retryAttempts;
        }

        return resource;
    };

    /**
     * Called when a resource fails to load. This will call the retryCallback function if defined until retryAttempts is reached.
     *
     * @param {Error} [error] The error that was encountered.
     *
     * @returns {Promise<Boolean>} A promise to a boolean, that if true will cause the resource request to be retried.
     *
     * @private
     */
    Resource.prototype.retryOnError = function(error) {
        var retryCallback = this.retryCallback;
        if ((typeof retryCallback !== 'function') || (this._retryCount >= this.retryAttempts)) {
            return when(false);
        }

        var that = this;
        return when(retryCallback(this, error))
            .then(function(result) {
                ++that._retryCount;

                return result;
            });
    };

    /**
     * Duplicates a Resource instance.
     *
     * @param {Resource} [result] The object onto which to store the result.
     *
     * @returns {Resource} The modified result parameter or a new Resource instance if one was not provided.
     */
    Resource.prototype.clone = function(result) {
        if (!defined(result)) {
            result = new Resource({
                url : this._url
            });
        }

        result._url = this._url;
        result._queryParameters = clone(this._queryParameters);
        result._templateValues = clone(this._templateValues);
        result.headers = clone(this.headers);
        result.proxy = this.proxy;
        result.retryCallback = this.retryCallback;
        result.retryAttempts = this.retryAttempts;
        result._retryCount = 0;
        result.request = this.request.clone();

        return result;
    };

    /**
     * Returns the base path of the Resource.
     *
     * @param {Boolean} [includeQuery = false] Whether or not to include the query string and fragment form the uri
     *
     * @returns {String} The base URI of the resource
     */
    Resource.prototype.getBaseUri = function(includeQuery) {
        return getBaseUri(this.getUrlComponent(includeQuery), includeQuery);
    };

    /**
     * Appends a forward slash to the URL.
     */
    Resource.prototype.appendForwardSlash = function() {
        this._url = appendForwardSlash(this._url);
    };

    /**
     * Asynchronously loads the resource as raw binary data.  Returns a promise that will resolve to
     * an ArrayBuffer once loaded, or reject if the resource failed to load.  The data is loaded
     * using XMLHttpRequest, which means that in order to make requests to another origin,
     * the server must have Cross-Origin Resource Sharing (CORS) headers enabled.
     *
     * @returns {Promise.<ArrayBuffer>|undefined} a promise that will resolve to the requested data when loaded. Returns undefined if <code>request.throttle</code> is true and the request does not have high enough priority.
     *
     * @example
     * // load a single URL asynchronously
     * resource.fetchArrayBuffer().then(function(arrayBuffer) {
     *     // use the data
     * }).otherwise(function(error) {
     *     // an error occurred
     * });
     *
     * @see {@link http://www.w3.org/TR/cors/|Cross-Origin Resource Sharing}
     * @see {@link http://wiki.commonjs.org/wiki/Promises/A|CommonJS Promises/A}
     */
    Resource.prototype.fetchArrayBuffer = function () {
        return this.fetch({
            responseType : 'arraybuffer'
        });
    };

    /**
     * Creates a Resource and calls fetchArrayBuffer() on it.
     *
     * @param {String|Object} options A url or an object with the following properties
     * @param {String} options.url The url of the resource.
     * @param {Object} [options.queryParameters] An object containing query parameters that will be sent when retrieving the resource.
     * @param {Object} [options.templateValues] Key/Value pairs that are used to replace template values (eg. {x}).
     * @param {Object} [options.headers={}] Additional HTTP headers that will be sent.
     * @param {DefaultProxy} [options.proxy] A proxy to be used when loading the resource.
     * @param {Resource~RetryCallback} [options.retryCallback] The Function to call when a request for this resource fails. If it returns true, the request will be retried.
     * @param {Number} [options.retryAttempts=0] The number of times the retryCallback should be called before giving up.
     * @param {Request} [options.request] A Request object that will be used. Intended for internal use only.
     * @returns {Promise.<ArrayBuffer>|undefined} a promise that will resolve to the requested data when loaded. Returns undefined if <code>request.throttle</code> is true and the request does not have high enough priority.
     */
    Resource.fetchArrayBuffer = function (options) {
        var resource = new Resource(options);
        return resource.fetchArrayBuffer();
    };

    /**
     * Asynchronously loads the given resource as a blob.  Returns a promise that will resolve to
     * a Blob once loaded, or reject if the resource failed to load.  The data is loaded
     * using XMLHttpRequest, which means that in order to make requests to another origin,
     * the server must have Cross-Origin Resource Sharing (CORS) headers enabled.
     *
     * @returns {Promise.<Blob>|undefined} a promise that will resolve to the requested data when loaded. Returns undefined if <code>request.throttle</code> is true and the request does not have high enough priority.
     *
     * @example
     * // load a single URL asynchronously
     * resource.fetchBlob().then(function(blob) {
     *     // use the data
     * }).otherwise(function(error) {
     *     // an error occurred
     * });
     *
     * @see {@link http://www.w3.org/TR/cors/|Cross-Origin Resource Sharing}
     * @see {@link http://wiki.commonjs.org/wiki/Promises/A|CommonJS Promises/A}
     */
    Resource.prototype.fetchBlob = function () {
        return this.fetch({
            responseType : 'blob'
        });
    };

    /**
     * Creates a Resource and calls fetchBlob() on it.
     *
     * @param {String|Object} options A url or an object with the following properties
     * @param {String} options.url The url of the resource.
     * @param {Object} [options.queryParameters] An object containing query parameters that will be sent when retrieving the resource.
     * @param {Object} [options.templateValues] Key/Value pairs that are used to replace template values (eg. {x}).
     * @param {Object} [options.headers={}] Additional HTTP headers that will be sent.
     * @param {DefaultProxy} [options.proxy] A proxy to be used when loading the resource.
     * @param {Resource~RetryCallback} [options.retryCallback] The Function to call when a request for this resource fails. If it returns true, the request will be retried.
     * @param {Number} [options.retryAttempts=0] The number of times the retryCallback should be called before giving up.
     * @param {Request} [options.request] A Request object that will be used. Intended for internal use only.
     * @returns {Promise.<Blob>|undefined} a promise that will resolve to the requested data when loaded. Returns undefined if <code>request.throttle</code> is true and the request does not have high enough priority.
     */
    Resource.fetchBlob = function (options) {
        var resource = new Resource(options);
        return resource.fetchBlob();
    };

    /**
     * Asynchronously loads the given image resource.  Returns a promise that will resolve to
     * an {@link https://developer.mozilla.org/en-US/docs/Web/API/ImageBitmap|ImageBitmap} if <code>preferImageBitmap</code> is true and the browser supports <code>createImageBitmap</code> or otherwise an
     * {@link https://developer.mozilla.org/en-US/docs/Web/API/HTMLImageElement|Image} once loaded, or reject if the image failed to load.
     *
     * @param {Object} [options] An object with the following properties.
     * @param {Boolean} [options.preferBlob=false] If true, we will load the image via a blob.
     * @param {Boolean} [options.preferImageBitmap=false] If true, image will be decoded during fetch and an <code>ImageBitmap</code> is returned.
     * @param {Boolean} [options.flipY=false] If true, image will be vertically flipped during decode. Only applies if the browser supports <code>createImageBitmap</code>.
     * @returns {Promise.<ImageBitmap>|Promise.<Image>|undefined} a promise that will resolve to the requested data when loaded. Returns undefined if <code>request.throttle</code> is true and the request does not have high enough priority.
     *
     *
     * @example
     * // load a single image asynchronously
     * resource.fetchImage().then(function(image) {
     *     // use the loaded image
     * }).otherwise(function(error) {
     *     // an error occurred
     * });
     *
     * // load several images in parallel
     * when.all([resource1.fetchImage(), resource2.fetchImage()]).then(function(images) {
     *     // images is an array containing all the loaded images
     * });
     *
     * @see {@link http://www.w3.org/TR/cors/|Cross-Origin Resource Sharing}
     * @see {@link http://wiki.commonjs.org/wiki/Promises/A|CommonJS Promises/A}
     */
    Resource.prototype.fetchImage = function (options) {
        options = defaultValue(options, defaultValue.EMPTY_OBJECT);
        var preferImageBitmap = defaultValue(options.preferImageBitmap, false);
        var preferBlob = defaultValue(options.preferBlob, false);
        var flipY = defaultValue(options.flipY, false);

        checkAndResetRequest(this.request);

        // We try to load the image normally if
        // 1. Blobs aren't supported
        // 2. It's a data URI
        // 3. It's a blob URI
        // 4. It doesn't have request headers and we preferBlob is false
        if (!xhrBlobSupported || this.isDataUri || this.isBlobUri || (!this.hasHeaders && !preferBlob)) {
            return fetchImage({
                resource: this,
                flipY: flipY,
                preferImageBitmap: preferImageBitmap
            });
        }

        var blobPromise = this.fetchBlob();
        if (!defined(blobPromise)) {
            return;
        }

        var supportsImageBitmap;
        var useImageBitmap;
        var generatedBlobResource;
        var generatedBlob;
        return Resource.supportsImageBitmapOptions()
            .then(function(result) {
                supportsImageBitmap = result;
                useImageBitmap = supportsImageBitmap && preferImageBitmap;
                return blobPromise;
            })
            .then(function(blob) {
                if (!defined(blob)) {
                    return;
                }
                generatedBlob = blob;
                if (useImageBitmap) {
                    return Resource.createImageBitmapFromBlob(blob, {
                        flipY: flipY,
                        premultiplyAlpha: false
                    });
                }
                var blobUrl = window.URL.createObjectURL(blob);
                generatedBlobResource = new Resource({
                    url: blobUrl
                });

                return fetchImage({
                    resource: generatedBlobResource,
                    flipY: flipY,
                    preferImageBitmap: false
                });
            })
            .then(function(image) {
                if (!defined(image)) {
                    return;
                }

                // The blob object may be needed for use by a TileDiscardPolicy,
                // so attach it to the image.
                image.blob = generatedBlob;

                if (useImageBitmap) {
                    return image;
                }

                window.URL.revokeObjectURL(generatedBlobResource.url);
                return image;
            })
            .otherwise(function(error) {
                if (defined(generatedBlobResource)) {
                    window.URL.revokeObjectURL(generatedBlobResource.url);
                }

                // If the blob load succeeded but the image decode failed, attach the blob
                // to the error object for use by a TileDiscardPolicy.
                // In particular, BingMapsImageryProvider uses this to detect the
                // zero-length response that is returned when a tile is not available.
                error.blob = generatedBlob;

                return when.reject(error);
            });
    };

    /**
     * Fetches an image and returns a promise to it.
     *
     * @param {Object} [options] An object with the following properties.
     * @param {Resource} [options.resource] Resource object that points to an image to fetch.
     * @param {Boolean} [options.preferImageBitmap] If true, image will be decoded during fetch and an <code>ImageBitmap</code> is returned.
     * @param {Boolean} [options.flipY] If true, image will be vertically flipped during decode. Only applies if the browser supports <code>createImageBitmap</code>.
     *
     * @private
     */
    function fetchImage(options) {
        var resource = options.resource;
        var flipY = options.flipY;
        var preferImageBitmap = options.preferImageBitmap;

        var request = resource.request;
        request.url = resource.url;
        request.requestFunction = function() {
            var url = resource.url;
            var crossOrigin = false;

            // data URIs can't have crossorigin set.
            if (!resource.isDataUri && !resource.isBlobUri) {
                crossOrigin = resource.isCrossOriginUrl;
            }

            var deferred = when.defer();
            Resource._Implementations.createImage(url, crossOrigin, deferred, flipY, preferImageBitmap);

            return deferred.promise;
        };

        var promise = RequestScheduler.request(request);
        if (!defined(promise)) {
            return;
        }

        return promise
            .otherwise(function(e) {
                // Don't retry cancelled or otherwise aborted requests
                if (request.state !== RequestState.FAILED) {
                    return when.reject(e);
                }

                return resource.retryOnError(e)
                    .then(function(retry) {
                        if (retry) {
                            // Reset request so it can try again
                            request.state = RequestState.UNISSUED;
                            request.deferred = undefined;

                            return fetchImage({
                                resource: resource,
                                flipY: flipY,
                                preferImageBitmap: preferImageBitmap
                            });
                        }

                        return when.reject(e);
                    });
            });
    }

    /**
     * Creates a Resource and calls fetchImage() on it.
     *
     * @param {String|Object} options A url or an object with the following properties
     * @param {String} options.url The url of the resource.
     * @param {Object} [options.queryParameters] An object containing query parameters that will be sent when retrieving the resource.
     * @param {Object} [options.templateValues] Key/Value pairs that are used to replace template values (eg. {x}).
     * @param {Object} [options.headers={}] Additional HTTP headers that will be sent.
     * @param {DefaultProxy} [options.proxy] A proxy to be used when loading the resource.
     * @param {Boolean} [options.flipY=false] Whether to vertically flip the image during fetch and decode. Only applies when requesting an image and the browser supports <code>createImageBitmap</code>.
     * @param {Resource~RetryCallback} [options.retryCallback] The Function to call when a request for this resource fails. If it returns true, the request will be retried.
     * @param {Number} [options.retryAttempts=0] The number of times the retryCallback should be called before giving up.
     * @param {Request} [options.request] A Request object that will be used. Intended for internal use only.
     * @param {Boolean} [options.preferBlob=false]  If true, we will load the image via a blob.
     * @param {Boolean} [options.preferImageBitmap=false] If true, image will be decoded during fetch and an <code>ImageBitmap</code> is returned.
     * @returns {Promise.<ImageBitmap>|Promise.<Image>|undefined} a promise that will resolve to the requested data when loaded. Returns undefined if <code>request.throttle</code> is true and the request does not have high enough priority.
     */
    Resource.fetchImage = function (options) {
        var resource = new Resource(options);
        return resource.fetchImage({
            flipY: options.flipY,
            preferBlob: options.preferBlob,
            preferImageBitmap: options.preferImageBitmap
        });
    };

    /**
     * Asynchronously loads the given resource as text.  Returns a promise that will resolve to
     * a String once loaded, or reject if the resource failed to load.  The data is loaded
     * using XMLHttpRequest, which means that in order to make requests to another origin,
     * the server must have Cross-Origin Resource Sharing (CORS) headers enabled.
     *
     * @returns {Promise.<String>|undefined} a promise that will resolve to the requested data when loaded. Returns undefined if <code>request.throttle</code> is true and the request does not have high enough priority.
     *
     * @example
     * // load text from a URL, setting a custom header
     * var resource = new Resource({
     *   url: 'http://someUrl.com/someJson.txt',
     *   headers: {
     *     'X-Custom-Header' : 'some value'
     *   }
     * });
     * resource.fetchText().then(function(text) {
     *     // Do something with the text
     * }).otherwise(function(error) {
     *     // an error occurred
     * });
     *
     * @see {@link https://developer.mozilla.org/en-US/docs/Web/API/XMLHttpRequest|XMLHttpRequest}
     * @see {@link http://www.w3.org/TR/cors/|Cross-Origin Resource Sharing}
     * @see {@link http://wiki.commonjs.org/wiki/Promises/A|CommonJS Promises/A}
     */
    Resource.prototype.fetchText = function() {
        return this.fetch({
            responseType : 'text'
        });
    };

    /**
     * Creates a Resource and calls fetchText() on it.
     *
     * @param {String|Object} options A url or an object with the following properties
     * @param {String} options.url The url of the resource.
     * @param {Object} [options.queryParameters] An object containing query parameters that will be sent when retrieving the resource.
     * @param {Object} [options.templateValues] Key/Value pairs that are used to replace template values (eg. {x}).
     * @param {Object} [options.headers={}] Additional HTTP headers that will be sent.
     * @param {DefaultProxy} [options.proxy] A proxy to be used when loading the resource.
     * @param {Resource~RetryCallback} [options.retryCallback] The Function to call when a request for this resource fails. If it returns true, the request will be retried.
     * @param {Number} [options.retryAttempts=0] The number of times the retryCallback should be called before giving up.
     * @param {Request} [options.request] A Request object that will be used. Intended for internal use only.
     * @returns {Promise.<String>|undefined} a promise that will resolve to the requested data when loaded. Returns undefined if <code>request.throttle</code> is true and the request does not have high enough priority.
     */
    Resource.fetchText = function (options) {
        var resource = new Resource(options);
        return resource.fetchText();
    };

    // note: &#42;&#47;&#42; below is */* but that ends the comment block early
    /**
     * Asynchronously loads the given resource as JSON.  Returns a promise that will resolve to
     * a JSON object once loaded, or reject if the resource failed to load.  The data is loaded
     * using XMLHttpRequest, which means that in order to make requests to another origin,
     * the server must have Cross-Origin Resource Sharing (CORS) headers enabled. This function
     * adds 'Accept: application/json,&#42;&#47;&#42;;q=0.01' to the request headers, if not
     * already specified.
     *
     * @returns {Promise.<Object>|undefined} a promise that will resolve to the requested data when loaded. Returns undefined if <code>request.throttle</code> is true and the request does not have high enough priority.
     *
     *
     * @example
     * resource.fetchJson().then(function(jsonData) {
     *     // Do something with the JSON object
     * }).otherwise(function(error) {
     *     // an error occurred
     * });
     *
     * @see {@link http://www.w3.org/TR/cors/|Cross-Origin Resource Sharing}
     * @see {@link http://wiki.commonjs.org/wiki/Promises/A|CommonJS Promises/A}
     */
    Resource.prototype.fetchJson = function() {
        var promise = this.fetch({
            responseType : 'text',
            headers: {
                Accept : 'application/json,*/*;q=0.01'
            }
        });

        if (!defined(promise)) {
            return undefined;
        }

        return promise
            .then(function(value) {
                if (!defined(value)) {
                    return;
                }
                return JSON.parse(value);
            });
    };

    /**
     * Creates a Resource and calls fetchJson() on it.
     *
     * @param {String|Object} options A url or an object with the following properties
     * @param {String} options.url The url of the resource.
     * @param {Object} [options.queryParameters] An object containing query parameters that will be sent when retrieving the resource.
     * @param {Object} [options.templateValues] Key/Value pairs that are used to replace template values (eg. {x}).
     * @param {Object} [options.headers={}] Additional HTTP headers that will be sent.
     * @param {DefaultProxy} [options.proxy] A proxy to be used when loading the resource.
     * @param {Resource~RetryCallback} [options.retryCallback] The Function to call when a request for this resource fails. If it returns true, the request will be retried.
     * @param {Number} [options.retryAttempts=0] The number of times the retryCallback should be called before giving up.
     * @param {Request} [options.request] A Request object that will be used. Intended for internal use only.
     * @returns {Promise.<Object>|undefined} a promise that will resolve to the requested data when loaded. Returns undefined if <code>request.throttle</code> is true and the request does not have high enough priority.
     */
    Resource.fetchJson = function (options) {
        var resource = new Resource(options);
        return resource.fetchJson();
    };

    /**
     * Asynchronously loads the given resource as XML.  Returns a promise that will resolve to
     * an XML Document once loaded, or reject if the resource failed to load.  The data is loaded
     * using XMLHttpRequest, which means that in order to make requests to another origin,
     * the server must have Cross-Origin Resource Sharing (CORS) headers enabled.
     *
     * @returns {Promise.<XMLDocument>|undefined} a promise that will resolve to the requested data when loaded. Returns undefined if <code>request.throttle</code> is true and the request does not have high enough priority.
     *
     *
     * @example
     * // load XML from a URL, setting a custom header
     * Cesium.loadXML('http://someUrl.com/someXML.xml', {
     *   'X-Custom-Header' : 'some value'
     * }).then(function(document) {
     *     // Do something with the document
     * }).otherwise(function(error) {
     *     // an error occurred
     * });
     *
     * @see {@link https://developer.mozilla.org/en-US/docs/Web/API/XMLHttpRequest|XMLHttpRequest}
     * @see {@link http://www.w3.org/TR/cors/|Cross-Origin Resource Sharing}
     * @see {@link http://wiki.commonjs.org/wiki/Promises/A|CommonJS Promises/A}
     */
    Resource.prototype.fetchXML = function() {
        return this.fetch({
            responseType : 'document',
            overrideMimeType : 'text/xml'
        });
    };

    /**
     * Creates a Resource and calls fetchXML() on it.
     *
     * @param {String|Object} options A url or an object with the following properties
     * @param {String} options.url The url of the resource.
     * @param {Object} [options.queryParameters] An object containing query parameters that will be sent when retrieving the resource.
     * @param {Object} [options.templateValues] Key/Value pairs that are used to replace template values (eg. {x}).
     * @param {Object} [options.headers={}] Additional HTTP headers that will be sent.
     * @param {DefaultProxy} [options.proxy] A proxy to be used when loading the resource.
     * @param {Resource~RetryCallback} [options.retryCallback] The Function to call when a request for this resource fails. If it returns true, the request will be retried.
     * @param {Number} [options.retryAttempts=0] The number of times the retryCallback should be called before giving up.
     * @param {Request} [options.request] A Request object that will be used. Intended for internal use only.
     * @returns {Promise.<XMLDocument>|undefined} a promise that will resolve to the requested data when loaded. Returns undefined if <code>request.throttle</code> is true and the request does not have high enough priority.
     */
    Resource.fetchXML = function (options) {
        var resource = new Resource(options);
        return resource.fetchXML();
    };

    /**
     * Requests a resource using JSONP.
     *
     * @param {String} [callbackParameterName='callback'] The callback parameter name that the server expects.
     * @returns {Promise.<Object>|undefined} a promise that will resolve to the requested data when loaded. Returns undefined if <code>request.throttle</code> is true and the request does not have high enough priority.
     *
     *
     * @example
     * // load a data asynchronously
     * resource.fetchJsonp().then(function(data) {
     *     // use the loaded data
     * }).otherwise(function(error) {
     *     // an error occurred
     * });
     *
     * @see {@link http://wiki.commonjs.org/wiki/Promises/A|CommonJS Promises/A}
     */
    Resource.prototype.fetchJsonp = function(callbackParameterName) {
        callbackParameterName = defaultValue(callbackParameterName, 'callback');

        checkAndResetRequest(this.request);

        //generate a unique function name
        var functionName;
        do {
            functionName = 'loadJsonp' + Math.random().toString().substring(2, 8);
        } while (defined(window[functionName]));

        return fetchJsonp(this, callbackParameterName, functionName);
    };

    function fetchJsonp(resource, callbackParameterName, functionName) {
        var callbackQuery = {};
        callbackQuery[callbackParameterName] = functionName;
        resource.setQueryParameters(callbackQuery);

        var request = resource.request;
        request.url = resource.url;
        request.requestFunction = function() {
            var deferred = when.defer();

            //assign a function with that name in the global scope
            window[functionName] = function(data) {
                deferred.resolve(data);

                try {
                    delete window[functionName];
                } catch (e) {
                    window[functionName] = undefined;
                }
            };

            Resource._Implementations.loadAndExecuteScript(resource.url, functionName, deferred);
            return deferred.promise;
        };

        var promise = RequestScheduler.request(request);
        if (!defined(promise)) {
            return;
        }

        return promise
            .otherwise(function(e) {
                if (request.state !== RequestState.FAILED) {
                    return when.reject(e);
                }

                return resource.retryOnError(e)
                    .then(function(retry) {
                        if (retry) {
                            // Reset request so it can try again
                            request.state = RequestState.UNISSUED;
                            request.deferred = undefined;

                            return fetchJsonp(resource, callbackParameterName, functionName);
                        }

                        return when.reject(e);
                    });
            });
    }

    /**
     * Creates a Resource from a URL and calls fetchJsonp() on it.
     *
     * @param {String|Object} options A url or an object with the following properties
     * @param {String} options.url The url of the resource.
     * @param {Object} [options.queryParameters] An object containing query parameters that will be sent when retrieving the resource.
     * @param {Object} [options.templateValues] Key/Value pairs that are used to replace template values (eg. {x}).
     * @param {Object} [options.headers={}] Additional HTTP headers that will be sent.
     * @param {DefaultProxy} [options.proxy] A proxy to be used when loading the resource.
     * @param {Resource~RetryCallback} [options.retryCallback] The Function to call when a request for this resource fails. If it returns true, the request will be retried.
     * @param {Number} [options.retryAttempts=0] The number of times the retryCallback should be called before giving up.
     * @param {Request} [options.request] A Request object that will be used. Intended for internal use only.
     * @param {String} [options.callbackParameterName='callback'] The callback parameter name that the server expects.
     * @returns {Promise.<Object>|undefined} a promise that will resolve to the requested data when loaded. Returns undefined if <code>request.throttle</code> is true and the request does not have high enough priority.
     */
    Resource.fetchJsonp = function (options) {
        var resource = new Resource(options);
        return resource.fetchJsonp(options.callbackParameterName);
    };

    /**
     * @private
     */
    Resource.prototype._makeRequest = function(options) {
        var resource = this;
        checkAndResetRequest(resource.request);

        var request = resource.request;
        request.url = resource.url;

        request.requestFunction = function() {
            var responseType = options.responseType;
            var headers = combine(options.headers, resource.headers);
            var overrideMimeType = options.overrideMimeType;
            var method = options.method;
            var data = options.data;
            var deferred = when.defer();
            var xhr = Resource._Implementations.loadWithXhr(resource.url, responseType, method, data, headers, deferred, overrideMimeType);
            if (defined(xhr) && defined(xhr.abort)) {
                request.cancelFunction = function() {
                    xhr.abort();
                };
            }
            return deferred.promise;
        };

        var promise = RequestScheduler.request(request);
        if (!defined(promise)) {
            return;
        }

        return promise
            .then(function(data) {
                return data;
            })
            .otherwise(function(e) {
                if (request.state !== RequestState.FAILED) {
                    return when.reject(e);
                }

                return resource.retryOnError(e)
                    .then(function(retry) {
                        if (retry) {
                            // Reset request so it can try again
                            request.state = RequestState.UNISSUED;
                            request.deferred = undefined;

                            return resource.fetch(options);
                        }

                        return when.reject(e);
                    });
            });
    };

    var dataUriRegex = /^data:(.*?)(;base64)?,(.*)$/;

    function decodeDataUriText(isBase64, data) {
        var result = decodeURIComponent(data);
        if (isBase64) {
            return atob(result);
        }
        return result;
    }

    function decodeDataUriArrayBuffer(isBase64, data) {
        var byteString = decodeDataUriText(isBase64, data);
        var buffer = new ArrayBuffer(byteString.length);
        var view = new Uint8Array(buffer);
        for (var i = 0; i < byteString.length; i++) {
            view[i] = byteString.charCodeAt(i);
        }
        return buffer;
    }

    function decodeDataUri(dataUriRegexResult, responseType) {
        responseType = defaultValue(responseType, '');
        var mimeType = dataUriRegexResult[1];
        var isBase64 = !!dataUriRegexResult[2];
        var data = dataUriRegexResult[3];

        switch (responseType) {
            case '':
            case 'text':
                return decodeDataUriText(isBase64, data);
            case 'arraybuffer':
                return decodeDataUriArrayBuffer(isBase64, data);
            case 'blob':
                var buffer = decodeDataUriArrayBuffer(isBase64, data);
                return new Blob([buffer], {
                    type : mimeType
                });
            case 'document':
                var parser = new DOMParser();
                return parser.parseFromString(decodeDataUriText(isBase64, data), mimeType);
            case 'json':
                return JSON.parse(decodeDataUriText(isBase64, data));
            default:
                                throw new DeveloperError('Unhandled responseType: ' + responseType);
                    }
    }

    /**
     * Asynchronously loads the given resource.  Returns a promise that will resolve to
     * the result once loaded, or reject if the resource failed to load.  The data is loaded
     * using XMLHttpRequest, which means that in order to make requests to another origin,
     * the server must have Cross-Origin Resource Sharing (CORS) headers enabled. It's recommended that you use
     * the more specific functions eg. fetchJson, fetchBlob, etc.
     *
     * @param {Object} [options] Object with the following properties:
     * @param {String} [options.responseType] The type of response.  This controls the type of item returned.
     * @param {Object} [options.headers] Additional HTTP headers to send with the request, if any.
     * @param {String} [options.overrideMimeType] Overrides the MIME type returned by the server.
     * @returns {Promise.<Object>|undefined} a promise that will resolve to the requested data when loaded. Returns undefined if <code>request.throttle</code> is true and the request does not have high enough priority.
     *
     *
     * @example
     * resource.fetch()
     *   .then(function(body) {
     *       // use the data
     *   }).otherwise(function(error) {
     *       // an error occurred
     *   });
     *
     * @see {@link http://www.w3.org/TR/cors/|Cross-Origin Resource Sharing}
     * @see {@link http://wiki.commonjs.org/wiki/Promises/A|CommonJS Promises/A}
     */
    Resource.prototype.fetch = function(options) {
        options = defaultClone(options, {});
        options.method = 'GET';

        return this._makeRequest(options);
    };

    /**
     * Creates a Resource from a URL and calls fetch() on it.
     *
     * @param {String|Object} options A url or an object with the following properties
     * @param {String} options.url The url of the resource.
     * @param {Object} [options.queryParameters] An object containing query parameters that will be sent when retrieving the resource.
     * @param {Object} [options.templateValues] Key/Value pairs that are used to replace template values (eg. {x}).
     * @param {Object} [options.headers={}] Additional HTTP headers that will be sent.
     * @param {DefaultProxy} [options.proxy] A proxy to be used when loading the resource.
     * @param {Resource~RetryCallback} [options.retryCallback] The Function to call when a request for this resource fails. If it returns true, the request will be retried.
     * @param {Number} [options.retryAttempts=0] The number of times the retryCallback should be called before giving up.
     * @param {Request} [options.request] A Request object that will be used. Intended for internal use only.
     * @param {String} [options.responseType] The type of response.  This controls the type of item returned.
     * @param {String} [options.overrideMimeType] Overrides the MIME type returned by the server.
     * @returns {Promise.<Object>|undefined} a promise that will resolve to the requested data when loaded. Returns undefined if <code>request.throttle</code> is true and the request does not have high enough priority.
     */
    Resource.fetch = function (options) {
        var resource = new Resource(options);
        return resource.fetch({
            // Make copy of just the needed fields because headers can be passed to both the constructor and to fetch
            responseType: options.responseType,
            overrideMimeType: options.overrideMimeType
        });
    };

    /**
     * Asynchronously deletes the given resource.  Returns a promise that will resolve to
     * the result once loaded, or reject if the resource failed to load.  The data is loaded
     * using XMLHttpRequest, which means that in order to make requests to another origin,
     * the server must have Cross-Origin Resource Sharing (CORS) headers enabled.
     *
     * @param {Object} [options] Object with the following properties:
     * @param {String} [options.responseType] The type of response.  This controls the type of item returned.
     * @param {Object} [options.headers] Additional HTTP headers to send with the request, if any.
     * @param {String} [options.overrideMimeType] Overrides the MIME type returned by the server.
     * @returns {Promise.<Object>|undefined} a promise that will resolve to the requested data when loaded. Returns undefined if <code>request.throttle</code> is true and the request does not have high enough priority.
     *
     *
     * @example
     * resource.delete()
     *   .then(function(body) {
     *       // use the data
     *   }).otherwise(function(error) {
     *       // an error occurred
     *   });
     *
     * @see {@link http://www.w3.org/TR/cors/|Cross-Origin Resource Sharing}
     * @see {@link http://wiki.commonjs.org/wiki/Promises/A|CommonJS Promises/A}
     */
    Resource.prototype.delete = function(options) {
        options = defaultClone(options, {});
        options.method = 'DELETE';

        return this._makeRequest(options);
    };

    /**
     * Creates a Resource from a URL and calls delete() on it.
     *
     * @param {String|Object} options A url or an object with the following properties
     * @param {String} options.url The url of the resource.
     * @param {Object} [options.data] Data that is posted with the resource.
     * @param {Object} [options.queryParameters] An object containing query parameters that will be sent when retrieving the resource.
     * @param {Object} [options.templateValues] Key/Value pairs that are used to replace template values (eg. {x}).
     * @param {Object} [options.headers={}] Additional HTTP headers that will be sent.
     * @param {DefaultProxy} [options.proxy] A proxy to be used when loading the resource.
     * @param {Resource~RetryCallback} [options.retryCallback] The Function to call when a request for this resource fails. If it returns true, the request will be retried.
     * @param {Number} [options.retryAttempts=0] The number of times the retryCallback should be called before giving up.
     * @param {Request} [options.request] A Request object that will be used. Intended for internal use only.
     * @param {String} [options.responseType] The type of response.  This controls the type of item returned.
     * @param {String} [options.overrideMimeType] Overrides the MIME type returned by the server.
     * @returns {Promise.<Object>|undefined} a promise that will resolve to the requested data when loaded. Returns undefined if <code>request.throttle</code> is true and the request does not have high enough priority.
     */
    Resource.delete = function (options) {
        var resource = new Resource(options);
        return resource.delete({
            // Make copy of just the needed fields because headers can be passed to both the constructor and to fetch
            responseType: options.responseType,
            overrideMimeType: options.overrideMimeType,
            data: options.data
        });
    };

    /**
     * Asynchronously gets headers the given resource.  Returns a promise that will resolve to
     * the result once loaded, or reject if the resource failed to load.  The data is loaded
     * using XMLHttpRequest, which means that in order to make requests to another origin,
     * the server must have Cross-Origin Resource Sharing (CORS) headers enabled.
     *
     * @param {Object} [options] Object with the following properties:
     * @param {String} [options.responseType] The type of response.  This controls the type of item returned.
     * @param {Object} [options.headers] Additional HTTP headers to send with the request, if any.
     * @param {String} [options.overrideMimeType] Overrides the MIME type returned by the server.
     * @returns {Promise.<Object>|undefined} a promise that will resolve to the requested data when loaded. Returns undefined if <code>request.throttle</code> is true and the request does not have high enough priority.
     *
     *
     * @example
     * resource.head()
     *   .then(function(headers) {
     *       // use the data
     *   }).otherwise(function(error) {
     *       // an error occurred
     *   });
     *
     * @see {@link http://www.w3.org/TR/cors/|Cross-Origin Resource Sharing}
     * @see {@link http://wiki.commonjs.org/wiki/Promises/A|CommonJS Promises/A}
     */
    Resource.prototype.head = function(options) {
        options = defaultClone(options, {});
        options.method = 'HEAD';

        return this._makeRequest(options);
    };

    /**
     * Creates a Resource from a URL and calls head() on it.
     *
     * @param {String|Object} options A url or an object with the following properties
     * @param {String} options.url The url of the resource.
     * @param {Object} [options.queryParameters] An object containing query parameters that will be sent when retrieving the resource.
     * @param {Object} [options.templateValues] Key/Value pairs that are used to replace template values (eg. {x}).
     * @param {Object} [options.headers={}] Additional HTTP headers that will be sent.
     * @param {DefaultProxy} [options.proxy] A proxy to be used when loading the resource.
     * @param {Resource~RetryCallback} [options.retryCallback] The Function to call when a request for this resource fails. If it returns true, the request will be retried.
     * @param {Number} [options.retryAttempts=0] The number of times the retryCallback should be called before giving up.
     * @param {Request} [options.request] A Request object that will be used. Intended for internal use only.
     * @param {String} [options.responseType] The type of response.  This controls the type of item returned.
     * @param {String} [options.overrideMimeType] Overrides the MIME type returned by the server.
     * @returns {Promise.<Object>|undefined} a promise that will resolve to the requested data when loaded. Returns undefined if <code>request.throttle</code> is true and the request does not have high enough priority.
     */
    Resource.head = function (options) {
        var resource = new Resource(options);
        return resource.head({
            // Make copy of just the needed fields because headers can be passed to both the constructor and to fetch
            responseType: options.responseType,
            overrideMimeType: options.overrideMimeType
        });
    };

    /**
     * Asynchronously gets options the given resource.  Returns a promise that will resolve to
     * the result once loaded, or reject if the resource failed to load.  The data is loaded
     * using XMLHttpRequest, which means that in order to make requests to another origin,
     * the server must have Cross-Origin Resource Sharing (CORS) headers enabled.
     *
     * @param {Object} [options] Object with the following properties:
     * @param {String} [options.responseType] The type of response.  This controls the type of item returned.
     * @param {Object} [options.headers] Additional HTTP headers to send with the request, if any.
     * @param {String} [options.overrideMimeType] Overrides the MIME type returned by the server.
     * @returns {Promise.<Object>|undefined} a promise that will resolve to the requested data when loaded. Returns undefined if <code>request.throttle</code> is true and the request does not have high enough priority.
     *
     *
     * @example
     * resource.options()
     *   .then(function(headers) {
     *       // use the data
     *   }).otherwise(function(error) {
     *       // an error occurred
     *   });
     *
     * @see {@link http://www.w3.org/TR/cors/|Cross-Origin Resource Sharing}
     * @see {@link http://wiki.commonjs.org/wiki/Promises/A|CommonJS Promises/A}
     */
    Resource.prototype.options = function(options) {
        options = defaultClone(options, {});
        options.method = 'OPTIONS';

        return this._makeRequest(options);
    };

    /**
     * Creates a Resource from a URL and calls options() on it.
     *
     * @param {String|Object} options A url or an object with the following properties
     * @param {String} options.url The url of the resource.
     * @param {Object} [options.queryParameters] An object containing query parameters that will be sent when retrieving the resource.
     * @param {Object} [options.templateValues] Key/Value pairs that are used to replace template values (eg. {x}).
     * @param {Object} [options.headers={}] Additional HTTP headers that will be sent.
     * @param {DefaultProxy} [options.proxy] A proxy to be used when loading the resource.
     * @param {Resource~RetryCallback} [options.retryCallback] The Function to call when a request for this resource fails. If it returns true, the request will be retried.
     * @param {Number} [options.retryAttempts=0] The number of times the retryCallback should be called before giving up.
     * @param {Request} [options.request] A Request object that will be used. Intended for internal use only.
     * @param {String} [options.responseType] The type of response.  This controls the type of item returned.
     * @param {String} [options.overrideMimeType] Overrides the MIME type returned by the server.
     * @returns {Promise.<Object>|undefined} a promise that will resolve to the requested data when loaded. Returns undefined if <code>request.throttle</code> is true and the request does not have high enough priority.
     */
    Resource.options = function (options) {
        var resource = new Resource(options);
        return resource.options({
            // Make copy of just the needed fields because headers can be passed to both the constructor and to fetch
            responseType: options.responseType,
            overrideMimeType: options.overrideMimeType
        });
    };

    /**
     * Asynchronously posts data to the given resource.  Returns a promise that will resolve to
     * the result once loaded, or reject if the resource failed to load.  The data is loaded
     * using XMLHttpRequest, which means that in order to make requests to another origin,
     * the server must have Cross-Origin Resource Sharing (CORS) headers enabled.
     *
     * @param {Object} data Data that is posted with the resource.
     * @param {Object} [options] Object with the following properties:
     * @param {Object} [options.data] Data that is posted with the resource.
     * @param {String} [options.responseType] The type of response.  This controls the type of item returned.
     * @param {Object} [options.headers] Additional HTTP headers to send with the request, if any.
     * @param {String} [options.overrideMimeType] Overrides the MIME type returned by the server.
     * @returns {Promise.<Object>|undefined} a promise that will resolve to the requested data when loaded. Returns undefined if <code>request.throttle</code> is true and the request does not have high enough priority.
     *
     *
     * @example
     * resource.post(data)
     *   .then(function(result) {
     *       // use the result
     *   }).otherwise(function(error) {
     *       // an error occurred
     *   });
     *
     * @see {@link http://www.w3.org/TR/cors/|Cross-Origin Resource Sharing}
     * @see {@link http://wiki.commonjs.org/wiki/Promises/A|CommonJS Promises/A}
     */
    Resource.prototype.post = function(data, options) {
        Check.defined('data', data);

        options = defaultClone(options, {});
        options.method = 'POST';
        options.data = data;

        return this._makeRequest(options);
    };

    /**
     * Creates a Resource from a URL and calls post() on it.
     *
     * @param {Object} options A url or an object with the following properties
     * @param {String} options.url The url of the resource.
     * @param {Object} options.data Data that is posted with the resource.
     * @param {Object} [options.queryParameters] An object containing query parameters that will be sent when retrieving the resource.
     * @param {Object} [options.templateValues] Key/Value pairs that are used to replace template values (eg. {x}).
     * @param {Object} [options.headers={}] Additional HTTP headers that will be sent.
     * @param {DefaultProxy} [options.proxy] A proxy to be used when loading the resource.
     * @param {Resource~RetryCallback} [options.retryCallback] The Function to call when a request for this resource fails. If it returns true, the request will be retried.
     * @param {Number} [options.retryAttempts=0] The number of times the retryCallback should be called before giving up.
     * @param {Request} [options.request] A Request object that will be used. Intended for internal use only.
     * @param {String} [options.responseType] The type of response.  This controls the type of item returned.
     * @param {String} [options.overrideMimeType] Overrides the MIME type returned by the server.
     * @returns {Promise.<Object>|undefined} a promise that will resolve to the requested data when loaded. Returns undefined if <code>request.throttle</code> is true and the request does not have high enough priority.
     */
    Resource.post = function (options) {
        var resource = new Resource(options);
        return resource.post(options.data, {
            // Make copy of just the needed fields because headers can be passed to both the constructor and to post
            responseType: options.responseType,
            overrideMimeType: options.overrideMimeType
        });
    };

    /**
     * Asynchronously puts data to the given resource.  Returns a promise that will resolve to
     * the result once loaded, or reject if the resource failed to load.  The data is loaded
     * using XMLHttpRequest, which means that in order to make requests to another origin,
     * the server must have Cross-Origin Resource Sharing (CORS) headers enabled.
     *
     * @param {Object} data Data that is posted with the resource.
     * @param {Object} [options] Object with the following properties:
     * @param {String} [options.responseType] The type of response.  This controls the type of item returned.
     * @param {Object} [options.headers] Additional HTTP headers to send with the request, if any.
     * @param {String} [options.overrideMimeType] Overrides the MIME type returned by the server.
     * @returns {Promise.<Object>|undefined} a promise that will resolve to the requested data when loaded. Returns undefined if <code>request.throttle</code> is true and the request does not have high enough priority.
     *
     *
     * @example
     * resource.put(data)
     *   .then(function(result) {
     *       // use the result
     *   }).otherwise(function(error) {
     *       // an error occurred
     *   });
     *
     * @see {@link http://www.w3.org/TR/cors/|Cross-Origin Resource Sharing}
     * @see {@link http://wiki.commonjs.org/wiki/Promises/A|CommonJS Promises/A}
     */
    Resource.prototype.put = function(data, options) {
        Check.defined('data', data);

        options = defaultClone(options, {});
        options.method = 'PUT';
        options.data = data;

        return this._makeRequest(options);
    };

    /**
     * Creates a Resource from a URL and calls put() on it.
     *
     * @param {Object} options A url or an object with the following properties
     * @param {String} options.url The url of the resource.
     * @param {Object} options.data Data that is posted with the resource.
     * @param {Object} [options.queryParameters] An object containing query parameters that will be sent when retrieving the resource.
     * @param {Object} [options.templateValues] Key/Value pairs that are used to replace template values (eg. {x}).
     * @param {Object} [options.headers={}] Additional HTTP headers that will be sent.
     * @param {DefaultProxy} [options.proxy] A proxy to be used when loading the resource.
     * @param {Resource~RetryCallback} [options.retryCallback] The Function to call when a request for this resource fails. If it returns true, the request will be retried.
     * @param {Number} [options.retryAttempts=0] The number of times the retryCallback should be called before giving up.
     * @param {Request} [options.request] A Request object that will be used. Intended for internal use only.
     * @param {String} [options.responseType] The type of response.  This controls the type of item returned.
     * @param {String} [options.overrideMimeType] Overrides the MIME type returned by the server.
     * @returns {Promise.<Object>|undefined} a promise that will resolve to the requested data when loaded. Returns undefined if <code>request.throttle</code> is true and the request does not have high enough priority.
     */
    Resource.put = function (options) {
        var resource = new Resource(options);
        return resource.put(options.data, {
            // Make copy of just the needed fields because headers can be passed to both the constructor and to post
            responseType: options.responseType,
            overrideMimeType: options.overrideMimeType
        });
    };

    /**
     * Asynchronously patches data to the given resource.  Returns a promise that will resolve to
     * the result once loaded, or reject if the resource failed to load.  The data is loaded
     * using XMLHttpRequest, which means that in order to make requests to another origin,
     * the server must have Cross-Origin Resource Sharing (CORS) headers enabled.
     *
     * @param {Object} data Data that is posted with the resource.
     * @param {Object} [options] Object with the following properties:
     * @param {String} [options.responseType] The type of response.  This controls the type of item returned.
     * @param {Object} [options.headers] Additional HTTP headers to send with the request, if any.
     * @param {String} [options.overrideMimeType] Overrides the MIME type returned by the server.
     * @returns {Promise.<Object>|undefined} a promise that will resolve to the requested data when loaded. Returns undefined if <code>request.throttle</code> is true and the request does not have high enough priority.
     *
     *
     * @example
     * resource.patch(data)
     *   .then(function(result) {
     *       // use the result
     *   }).otherwise(function(error) {
     *       // an error occurred
     *   });
     *
     * @see {@link http://www.w3.org/TR/cors/|Cross-Origin Resource Sharing}
     * @see {@link http://wiki.commonjs.org/wiki/Promises/A|CommonJS Promises/A}
     */
    Resource.prototype.patch = function(data, options) {
        Check.defined('data', data);

        options = defaultClone(options, {});
        options.method = 'PATCH';
        options.data = data;

        return this._makeRequest(options);
    };

    /**
     * Creates a Resource from a URL and calls patch() on it.
     *
     * @param {Object} options A url or an object with the following properties
     * @param {String} options.url The url of the resource.
     * @param {Object} options.data Data that is posted with the resource.
     * @param {Object} [options.queryParameters] An object containing query parameters that will be sent when retrieving the resource.
     * @param {Object} [options.templateValues] Key/Value pairs that are used to replace template values (eg. {x}).
     * @param {Object} [options.headers={}] Additional HTTP headers that will be sent.
     * @param {DefaultProxy} [options.proxy] A proxy to be used when loading the resource.
     * @param {Resource~RetryCallback} [options.retryCallback] The Function to call when a request for this resource fails. If it returns true, the request will be retried.
     * @param {Number} [options.retryAttempts=0] The number of times the retryCallback should be called before giving up.
     * @param {Request} [options.request] A Request object that will be used. Intended for internal use only.
     * @param {String} [options.responseType] The type of response.  This controls the type of item returned.
     * @param {String} [options.overrideMimeType] Overrides the MIME type returned by the server.
     * @returns {Promise.<Object>|undefined} a promise that will resolve to the requested data when loaded. Returns undefined if <code>request.throttle</code> is true and the request does not have high enough priority.
     */
    Resource.patch = function (options) {
        var resource = new Resource(options);
        return resource.patch(options.data, {
            // Make copy of just the needed fields because headers can be passed to both the constructor and to post
            responseType: options.responseType,
            overrideMimeType: options.overrideMimeType
        });
    };

    /**
     * Contains implementations of functions that can be replaced for testing
     *
     * @private
     */
    Resource._Implementations = {};

    function loadImageElement(url, crossOrigin, deferred) {
        var image = new Image();

        image.onload = function() {
            deferred.resolve(image);
        };

        image.onerror = function(e) {
            deferred.reject(e);
        };

        if (crossOrigin) {
            if (TrustedServers.contains(url)) {
                image.crossOrigin = 'use-credentials';
            } else {
                image.crossOrigin = '';
            }
        }

        image.src = url;
    }

    Resource._Implementations.createImage = function(url, crossOrigin, deferred, flipY, preferImageBitmap) {
        // Passing an Image to createImageBitmap will force it to run on the main thread
        // since DOM elements don't exist on workers. We convert it to a blob so it's non-blocking.
        // See:
        //    https://bugzilla.mozilla.org/show_bug.cgi?id=1044102#c38
        //    https://bugs.chromium.org/p/chromium/issues/detail?id=580202#c10
        Resource.supportsImageBitmapOptions()
            .then(function(supportsImageBitmap) {
                // We can only use ImageBitmap if we can flip on decode.
                // See: https://github.com/AnalyticalGraphicsInc/cesium/pull/7579#issuecomment-466146898
                if (!(supportsImageBitmap && preferImageBitmap)) {
                    loadImageElement(url, crossOrigin, deferred);
                    return;
                }

                return Resource.fetchBlob({
                    url: url
                })
                .then(function(blob) {
                    if (!defined(blob)) {
                        deferred.reject(new RuntimeError('Successfully retrieved ' + url + ' but it contained no content.'));
                        return;
                    }

                    return Resource.createImageBitmapFromBlob(blob, {
                        flipY: flipY,
                        premultiplyAlpha: false
                    });
                }).then(deferred.resolve);
            })
            .otherwise(deferred.reject);
    };

    /**
     * Wrapper for createImageBitmap
     *
     * @private
     */
    Resource.createImageBitmapFromBlob = function(blob, options) {
        Check.defined('options', options);
        Check.typeOf.bool('options.flipY', options.flipY);
        Check.typeOf.bool('options.premultiplyAlpha', options.premultiplyAlpha);

        return createImageBitmap(blob, {
            imageOrientation: options.flipY ? 'flipY' : 'none',
            premultiplyAlpha: options.premultiplyAlpha ? 'premultiply' : 'none'
        });
    };

    function decodeResponse(loadWithHttpResponse, responseType) {
        switch (responseType) {
          case 'text':
              return loadWithHttpResponse.toString('utf8');
          case 'json':
              return JSON.parse(loadWithHttpResponse.toString('utf8'));
          default:
              return new Uint8Array(loadWithHttpResponse).buffer;
        }
    }

    function loadWithHttpRequest(url, responseType, method, data, headers, deferred, overrideMimeType) {

        // Specifically use the Node version of require to avoid conflicts with the global
        // require defined in the built version of Cesium.
        var nodeRequire = global.require; // eslint-disable-line

        // Note: only the 'json' and 'text' responseTypes transforms the loaded buffer
        var URL = nodeRequire('url').parse(url);
        var http = URL.protocol === 'https:' ? nodeRequire('https') : nodeRequire('http');
        var zlib = nodeRequire('zlib');
        var options = {
            protocol : URL.protocol,
            hostname : URL.hostname,
            port : URL.port,
            path : URL.path,
            query : URL.query,
            method : method,
            headers : headers
        };

        http.request(options)
            .on('response', function(res) {
                if (res.statusCode < 200 || res.statusCode >= 300) {
                    deferred.reject(new RequestErrorEvent(res.statusCode, res, res.headers));
                    return;
                }

                var chunkArray = [];
                res.on('data', function(chunk) {
                    chunkArray.push(chunk);
                });

                res.on('end', function() {
                    var result = Buffer.concat(chunkArray); // eslint-disable-line
                    if (res.headers['content-encoding'] === 'gzip') {
                        zlib.gunzip(result, function(error, resultUnzipped) {
                            if (error) {
                                deferred.reject(new RuntimeError('Error decompressing response.'));
                            } else {
                                deferred.resolve(decodeResponse(resultUnzipped, responseType));
                            }
                        });
                    } else {
                        deferred.resolve(decodeResponse(result, responseType));
                    }
                });
            }).on('error', function(e) {
                deferred.reject(new RequestErrorEvent());
            }).end();
    }

    var noXMLHttpRequest = typeof XMLHttpRequest === 'undefined';
    Resource._Implementations.loadWithXhr = function(url, responseType, method, data, headers, deferred, overrideMimeType) {
        var dataUriRegexResult = dataUriRegex.exec(url);
        if (dataUriRegexResult !== null) {
            deferred.resolve(decodeDataUri(dataUriRegexResult, responseType));
            return;
        }

        if (noXMLHttpRequest) {
            loadWithHttpRequest(url, responseType, method, data, headers, deferred, overrideMimeType);
            return;
        }

        var xhr = new XMLHttpRequest();

        if (TrustedServers.contains(url)) {
            xhr.withCredentials = true;
        }

        xhr.open(method, url, true);

        if (defined(overrideMimeType) && defined(xhr.overrideMimeType)) {
            xhr.overrideMimeType(overrideMimeType);
        }

        if (defined(headers)) {
            for (var key in headers) {
                if (headers.hasOwnProperty(key)) {
                    xhr.setRequestHeader(key, headers[key]);
                }
            }
        }

        if (defined(responseType)) {
            xhr.responseType = responseType;
        }

        // While non-standard, file protocol always returns a status of 0 on success
        var localFile = false;
        if (typeof url === 'string') {
            localFile = (url.indexOf('file://') === 0) || (typeof window !== 'undefined' && window.location.origin === 'file://');
        }

        xhr.onload = function() {
            if ((xhr.status < 200 || xhr.status >= 300) && !(localFile && xhr.status === 0)) {
                deferred.reject(new RequestErrorEvent(xhr.status, xhr.response, xhr.getAllResponseHeaders()));
                return;
            }

            var response = xhr.response;
            var browserResponseType = xhr.responseType;

            if (method === 'HEAD' || method === 'OPTIONS') {
                var responseHeaderString = xhr.getAllResponseHeaders();
                var splitHeaders = responseHeaderString.trim().split(/[\r\n]+/);

                var responseHeaders = {};
                splitHeaders.forEach(function (line) {
                    var parts = line.split(': ');
                    var header = parts.shift();
                    responseHeaders[header] = parts.join(': ');
                });

                deferred.resolve(responseHeaders);
                return;
            }

            //All modern browsers will go into either the first or second if block or last else block.
            //Other code paths support older browsers that either do not support the supplied responseType
            //or do not support the xhr.response property.
            if (xhr.status === 204) {
                // accept no content
                deferred.resolve();
            } else if (defined(response) && (!defined(responseType) || (browserResponseType === responseType))) {
                deferred.resolve(response);
            } else if ((responseType === 'json') && typeof response === 'string') {
                try {
                    deferred.resolve(JSON.parse(response));
                } catch (e) {
                    deferred.reject(e);
                }
            } else if ((browserResponseType === '' || browserResponseType === 'document') && defined(xhr.responseXML) && xhr.responseXML.hasChildNodes()) {
                deferred.resolve(xhr.responseXML);
            } else if ((browserResponseType === '' || browserResponseType === 'text') && defined(xhr.responseText)) {
                deferred.resolve(xhr.responseText);
            } else {
                deferred.reject(new RuntimeError('Invalid XMLHttpRequest response type.'));
            }
        };

        xhr.onerror = function(e) {
            deferred.reject(new RequestErrorEvent());
        };

        xhr.send(data);

        return xhr;
    };

    Resource._Implementations.loadAndExecuteScript = function(url, functionName, deferred) {
        return loadAndExecuteScript(url, functionName).otherwise(deferred.reject);
    };

    /**
     * The default implementations
     *
     * @private
     */
    Resource._DefaultImplementations = {};
    Resource._DefaultImplementations.createImage = Resource._Implementations.createImage;
    Resource._DefaultImplementations.loadWithXhr = Resource._Implementations.loadWithXhr;
    Resource._DefaultImplementations.loadAndExecuteScript = Resource._Implementations.loadAndExecuteScript;

    /**
     * A resource instance initialized to the current browser location
     *
     * @type {Resource}
     * @constant
     */
    Resource.DEFAULT = freezeObject(new Resource({
        url: (typeof document === 'undefined') ? '' : document.location.href.split('?')[0]
    }));

    /**
     * A function that returns the value of the property.
     * @callback Resource~RetryCallback
     *
     * @param {Resource} [resource] The resource that failed to load.
     * @param {Error} [error] The error that occurred during the loading of the resource.
     * @returns {Boolean|Promise<Boolean>} If true or a promise that resolved to true, the resource will be retried. Otherwise the failure will be returned.
     */

    return Resource;
});

define('Core/EarthOrientationParameters',[
        '../ThirdParty/when',
        './binarySearch',
        './defaultValue',
        './defined',
        './EarthOrientationParametersSample',
        './freezeObject',
        './JulianDate',
        './LeapSecond',
        './Resource',
        './RuntimeError',
        './TimeConstants',
        './TimeStandard'
    ], function(
        when,
        binarySearch,
        defaultValue,
        defined,
        EarthOrientationParametersSample,
        freezeObject,
        JulianDate,
        LeapSecond,
        Resource,
        RuntimeError,
        TimeConstants,
        TimeStandard) {
    'use strict';

    /**
     * Specifies Earth polar motion coordinates and the difference between UT1 and UTC.
     * These Earth Orientation Parameters (EOP) are primarily used in the transformation from
     * the International Celestial Reference Frame (ICRF) to the International Terrestrial
     * Reference Frame (ITRF).
     *
     * @alias EarthOrientationParameters
     * @constructor
     *
     * @param {Object} [options] Object with the following properties:
     * @param {Resource|String} [options.url] The URL from which to obtain EOP data.  If neither this
     *                 parameter nor options.data is specified, all EOP values are assumed
     *                 to be 0.0.  If options.data is specified, this parameter is
     *                 ignored.
     * @param {Object} [options.data] The actual EOP data.  If neither this
     *                 parameter nor options.data is specified, all EOP values are assumed
     *                 to be 0.0.
     * @param {Boolean} [options.addNewLeapSeconds=true] True if leap seconds that
     *                  are specified in the EOP data but not in {@link JulianDate.leapSeconds}
     *                  should be added to {@link JulianDate.leapSeconds}.  False if
     *                  new leap seconds should be handled correctly in the context
     *                  of the EOP data but otherwise ignored.
     *
     * @example
     * // An example EOP data file, EOP.json:
     * {
     *   "columnNames" : ["dateIso8601","modifiedJulianDateUtc","xPoleWanderRadians","yPoleWanderRadians","ut1MinusUtcSeconds","lengthOfDayCorrectionSeconds","xCelestialPoleOffsetRadians","yCelestialPoleOffsetRadians","taiMinusUtcSeconds"],
     *   "samples" : [
     *      "2011-07-01T00:00:00Z",55743.0,2.117957047295119e-7,2.111518721609984e-6,-0.2908948,-2.956e-4,3.393695767766752e-11,3.3452143996557983e-10,34.0,
     *      "2011-07-02T00:00:00Z",55744.0,2.193297093339541e-7,2.115460256837405e-6,-0.29065,-1.824e-4,-8.241832578862112e-11,5.623838700870617e-10,34.0,
     *      "2011-07-03T00:00:00Z",55745.0,2.262286080161428e-7,2.1191157519929706e-6,-0.2905572,1.9e-6,-3.490658503988659e-10,6.981317007977318e-10,34.0
     *   ]
     * }
     *
     * @example
     * // Loading the EOP data
     * var eop = new Cesium.EarthOrientationParameters({ url : 'Data/EOP.json' });
     * Cesium.Transforms.earthOrientationParameters = eop;
     *
     * @private
     */
    function EarthOrientationParameters(options) {
        options = defaultValue(options, defaultValue.EMPTY_OBJECT);

        this._dates = undefined;
        this._samples = undefined;

        this._dateColumn = -1;
        this._xPoleWanderRadiansColumn = -1;
        this._yPoleWanderRadiansColumn = -1;
        this._ut1MinusUtcSecondsColumn = -1;
        this._xCelestialPoleOffsetRadiansColumn = -1;
        this._yCelestialPoleOffsetRadiansColumn = -1;
        this._taiMinusUtcSecondsColumn = -1;

        this._columnCount = 0;
        this._lastIndex = -1;

        this._downloadPromise = undefined;
        this._dataError = undefined;

        this._addNewLeapSeconds = defaultValue(options.addNewLeapSeconds, true);

        if (defined(options.data)) {
            // Use supplied EOP data.
            onDataReady(this, options.data);
        } else if (defined(options.url)) {
            var resource = Resource.createIfNeeded(options.url);

            // Download EOP data.
            var that = this;
            this._downloadPromise = when(resource.fetchJson(), function(eopData) {
                onDataReady(that, eopData);
            }, function() {
                that._dataError = 'An error occurred while retrieving the EOP data from the URL ' + resource.url + '.';
            });
        } else {
            // Use all zeros for EOP data.
            onDataReady(this, {
                'columnNames' : ['dateIso8601', 'modifiedJulianDateUtc', 'xPoleWanderRadians', 'yPoleWanderRadians', 'ut1MinusUtcSeconds', 'lengthOfDayCorrectionSeconds', 'xCelestialPoleOffsetRadians', 'yCelestialPoleOffsetRadians', 'taiMinusUtcSeconds'],
                'samples' : []
            });
        }
    }

    /**
     * A default {@link EarthOrientationParameters} instance that returns zero for all EOP values.
     */
    EarthOrientationParameters.NONE = freezeObject({
            getPromiseToLoad : function() {
                return when();
            },
            compute : function(date, result) {
                if (!defined(result)) {
                    result = new EarthOrientationParametersSample(0.0, 0.0, 0.0, 0.0, 0.0);
                } else {
                    result.xPoleWander = 0.0;
                    result.yPoleWander = 0.0;
                    result.xPoleOffset = 0.0;
                    result.yPoleOffset = 0.0;
                    result.ut1MinusUtc = 0.0;
                }
                return result;
            }
    });

    /**
     * Gets a promise that, when resolved, indicates that the EOP data has been loaded and is
     * ready to use.
     *
     * @returns {Promise} The promise.
     *
     * @see when
     */
    EarthOrientationParameters.prototype.getPromiseToLoad = function() {
        return when(this._downloadPromise);
    };

    /**
     * Computes the Earth Orientation Parameters (EOP) for a given date by interpolating.
     * If the EOP data has not yet been download, this method returns undefined.
     *
     * @param {JulianDate} date The date for each to evaluate the EOP.
     * @param {EarthOrientationParametersSample} [result] The instance to which to copy the result.
     *        If this parameter is undefined, a new instance is created and returned.
     * @returns {EarthOrientationParametersSample} The EOP evaluated at the given date, or
     *          undefined if the data necessary to evaluate EOP at the date has not yet been
     *          downloaded.
     *
     * @exception {RuntimeError} The loaded EOP data has an error and cannot be used.
     *
     * @see EarthOrientationParameters#getPromiseToLoad
     */
    EarthOrientationParameters.prototype.compute = function(date, result) {
        // We cannot compute until the samples are available.
        if (!defined(this._samples)) {
            if (defined(this._dataError)) {
                throw new RuntimeError(this._dataError);
            }

            return undefined;
        }

        if (!defined(result)) {
            result = new EarthOrientationParametersSample(0.0, 0.0, 0.0, 0.0, 0.0);
        }

        if (this._samples.length === 0) {
            result.xPoleWander = 0.0;
            result.yPoleWander = 0.0;
            result.xPoleOffset = 0.0;
            result.yPoleOffset = 0.0;
            result.ut1MinusUtc = 0.0;
            return result;
        }

        var dates = this._dates;
        var lastIndex = this._lastIndex;

        var before = 0;
        var after = 0;
        if (defined(lastIndex)) {
            var previousIndexDate = dates[lastIndex];
            var nextIndexDate = dates[lastIndex + 1];
            var isAfterPrevious = JulianDate.lessThanOrEquals(previousIndexDate, date);
            var isAfterLastSample = !defined(nextIndexDate);
            var isBeforeNext = isAfterLastSample || JulianDate.greaterThanOrEquals(nextIndexDate, date);

            if (isAfterPrevious && isBeforeNext) {
                before = lastIndex;

                if (!isAfterLastSample && nextIndexDate.equals(date)) {
                    ++before;
                }
                after = before + 1;

                interpolate(this, dates, this._samples, date, before, after, result);
                return result;
            }
        }

        var index = binarySearch(dates, date, JulianDate.compare, this._dateColumn);
        if (index >= 0) {
            // If the next entry is the same date, use the later entry.  This way, if two entries
            // describe the same moment, one before a leap second and the other after, then we will use
            // the post-leap second data.
            if (index < dates.length - 1 && dates[index + 1].equals(date)) {
                ++index;
            }
            before = index;
            after = index;
        } else {
            after = ~index;
            before = after - 1;

            // Use the first entry if the date requested is before the beginning of the data.
            if (before < 0) {
                before = 0;
            }
        }

        this._lastIndex = before;

        interpolate(this, dates, this._samples, date, before, after, result);
        return result;
    };

    function compareLeapSecondDates(leapSecond, dateToFind) {
        return JulianDate.compare(leapSecond.julianDate, dateToFind);
    }

    function onDataReady(eop, eopData) {
        if (!defined(eopData.columnNames)) {
            eop._dataError = 'Error in loaded EOP data: The columnNames property is required.';
            return;
        }

        if (!defined(eopData.samples)) {
            eop._dataError = 'Error in loaded EOP data: The samples property is required.';
            return;
        }

        var dateColumn = eopData.columnNames.indexOf('modifiedJulianDateUtc');
        var xPoleWanderRadiansColumn = eopData.columnNames.indexOf('xPoleWanderRadians');
        var yPoleWanderRadiansColumn = eopData.columnNames.indexOf('yPoleWanderRadians');
        var ut1MinusUtcSecondsColumn = eopData.columnNames.indexOf('ut1MinusUtcSeconds');
        var xCelestialPoleOffsetRadiansColumn = eopData.columnNames.indexOf('xCelestialPoleOffsetRadians');
        var yCelestialPoleOffsetRadiansColumn = eopData.columnNames.indexOf('yCelestialPoleOffsetRadians');
        var taiMinusUtcSecondsColumn = eopData.columnNames.indexOf('taiMinusUtcSeconds');

        if (dateColumn < 0 || xPoleWanderRadiansColumn < 0 || yPoleWanderRadiansColumn < 0 || ut1MinusUtcSecondsColumn < 0 || xCelestialPoleOffsetRadiansColumn < 0 || yCelestialPoleOffsetRadiansColumn < 0 || taiMinusUtcSecondsColumn < 0) {
            eop._dataError = 'Error in loaded EOP data: The columnNames property must include modifiedJulianDateUtc, xPoleWanderRadians, yPoleWanderRadians, ut1MinusUtcSeconds, xCelestialPoleOffsetRadians, yCelestialPoleOffsetRadians, and taiMinusUtcSeconds columns';
            return;
        }

        var samples = eop._samples = eopData.samples;
        var dates = eop._dates = [];

        eop._dateColumn = dateColumn;
        eop._xPoleWanderRadiansColumn = xPoleWanderRadiansColumn;
        eop._yPoleWanderRadiansColumn = yPoleWanderRadiansColumn;
        eop._ut1MinusUtcSecondsColumn = ut1MinusUtcSecondsColumn;
        eop._xCelestialPoleOffsetRadiansColumn = xCelestialPoleOffsetRadiansColumn;
        eop._yCelestialPoleOffsetRadiansColumn = yCelestialPoleOffsetRadiansColumn;
        eop._taiMinusUtcSecondsColumn = taiMinusUtcSecondsColumn;

        eop._columnCount = eopData.columnNames.length;
        eop._lastIndex = undefined;

        var lastTaiMinusUtc;

        var addNewLeapSeconds = eop._addNewLeapSeconds;

        // Convert the ISO8601 dates to JulianDates.
        for (var i = 0, len = samples.length; i < len; i += eop._columnCount) {
            var mjd = samples[i + dateColumn];
            var taiMinusUtc = samples[i + taiMinusUtcSecondsColumn];
            var day = mjd + TimeConstants.MODIFIED_JULIAN_DATE_DIFFERENCE;
            var date = new JulianDate(day, taiMinusUtc, TimeStandard.TAI);
            dates.push(date);

            if (addNewLeapSeconds) {
                if (taiMinusUtc !== lastTaiMinusUtc && defined(lastTaiMinusUtc)) {
                    // We crossed a leap second boundary, so add the leap second
                    // if it does not already exist.
                    var leapSeconds = JulianDate.leapSeconds;
                    var leapSecondIndex = binarySearch(leapSeconds, date, compareLeapSecondDates);
                    if (leapSecondIndex < 0) {
                        var leapSecond = new LeapSecond(date, taiMinusUtc);
                        leapSeconds.splice(~leapSecondIndex, 0, leapSecond);
                    }
                }
                lastTaiMinusUtc = taiMinusUtc;
            }
        }
    }

    function fillResultFromIndex(eop, samples, index, columnCount, result) {
        var start = index * columnCount;
        result.xPoleWander = samples[start + eop._xPoleWanderRadiansColumn];
        result.yPoleWander = samples[start + eop._yPoleWanderRadiansColumn];
        result.xPoleOffset = samples[start + eop._xCelestialPoleOffsetRadiansColumn];
        result.yPoleOffset = samples[start + eop._yCelestialPoleOffsetRadiansColumn];
        result.ut1MinusUtc = samples[start + eop._ut1MinusUtcSecondsColumn];
    }

    function linearInterp(dx, y1, y2) {
        return y1 + dx * (y2 - y1);
    }

    function interpolate(eop, dates, samples, date, before, after, result) {
        var columnCount = eop._columnCount;

        // First check the bounds on the EOP data
        // If we are after the bounds of the data, return zeros.
        // The 'before' index should never be less than zero.
        if (after > dates.length - 1) {
            result.xPoleWander = 0;
            result.yPoleWander = 0;
            result.xPoleOffset = 0;
            result.yPoleOffset = 0;
            result.ut1MinusUtc = 0;
            return result;
        }

        var beforeDate = dates[before];
        var afterDate = dates[after];
        if (beforeDate.equals(afterDate) || date.equals(beforeDate)) {
            fillResultFromIndex(eop, samples, before, columnCount, result);
            return result;
        } else if (date.equals(afterDate)) {
            fillResultFromIndex(eop, samples, after, columnCount, result);
            return result;
        }

        var factor = JulianDate.secondsDifference(date, beforeDate) / JulianDate.secondsDifference(afterDate, beforeDate);

        var startBefore = before * columnCount;
        var startAfter = after * columnCount;

        // Handle UT1 leap second edge case
        var beforeUt1MinusUtc = samples[startBefore + eop._ut1MinusUtcSecondsColumn];
        var afterUt1MinusUtc = samples[startAfter + eop._ut1MinusUtcSecondsColumn];

        var offsetDifference = afterUt1MinusUtc - beforeUt1MinusUtc;
        if (offsetDifference > 0.5 || offsetDifference < -0.5) {
            // The absolute difference between the values is more than 0.5, so we may have
            // crossed a leap second.  Check if this is the case and, if so, adjust the
            // afterValue to account for the leap second.  This way, our interpolation will
            // produce reasonable results.
            var beforeTaiMinusUtc = samples[startBefore + eop._taiMinusUtcSecondsColumn];
            var afterTaiMinusUtc = samples[startAfter + eop._taiMinusUtcSecondsColumn];
            if (beforeTaiMinusUtc !== afterTaiMinusUtc) {
                if (afterDate.equals(date)) {
                    // If we are at the end of the leap second interval, take the second value
                    // Otherwise, the interpolation below will yield the wrong side of the
                    // discontinuity
                    // At the end of the leap second, we need to start accounting for the jump
                    beforeUt1MinusUtc = afterUt1MinusUtc;
                } else {
                    // Otherwise, remove the leap second so that the interpolation is correct
                    afterUt1MinusUtc -= afterTaiMinusUtc - beforeTaiMinusUtc;
                }
            }
        }

        result.xPoleWander = linearInterp(factor, samples[startBefore + eop._xPoleWanderRadiansColumn], samples[startAfter + eop._xPoleWanderRadiansColumn]);
        result.yPoleWander = linearInterp(factor, samples[startBefore + eop._yPoleWanderRadiansColumn], samples[startAfter + eop._yPoleWanderRadiansColumn]);
        result.xPoleOffset = linearInterp(factor, samples[startBefore + eop._xCelestialPoleOffsetRadiansColumn], samples[startAfter + eop._xCelestialPoleOffsetRadiansColumn]);
        result.yPoleOffset = linearInterp(factor, samples[startBefore + eop._yCelestialPoleOffsetRadiansColumn], samples[startAfter + eop._yCelestialPoleOffsetRadiansColumn]);
        result.ut1MinusUtc = linearInterp(factor, beforeUt1MinusUtc, afterUt1MinusUtc);
        return result;
    }

    return EarthOrientationParameters;
});

define('Core/HeadingPitchRoll',[
        './defaultValue',
        './defined',
        './DeveloperError',
        './Math'
    ], function(
        defaultValue,
        defined,
        DeveloperError,
        CesiumMath) {
    'use strict';

    /**
     * A rotation expressed as a heading, pitch, and roll. Heading is the rotation about the
     * negative z axis. Pitch is the rotation about the negative y axis. Roll is the rotation about
     * the positive x axis.
     * @alias HeadingPitchRoll
     * @constructor
     *
     * @param {Number} [heading=0.0] The heading component in radians.
     * @param {Number} [pitch=0.0] The pitch component in radians.
     * @param {Number} [roll=0.0] The roll component in radians.
     */
    function HeadingPitchRoll(heading, pitch, roll) {
        this.heading = defaultValue(heading, 0.0);
        this.pitch = defaultValue(pitch, 0.0);
        this.roll = defaultValue(roll, 0.0);
    }

    /**
     * Computes the heading, pitch and roll from a quaternion (see http://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles )
     *
     * @param {Quaternion} quaternion The quaternion from which to retrieve heading, pitch, and roll, all expressed in radians.
     * @param {HeadingPitchRoll} [result] The object in which to store the result. If not provided, a new instance is created and returned.
     * @returns {HeadingPitchRoll} The modified result parameter or a new HeadingPitchRoll instance if one was not provided.
     */
    HeadingPitchRoll.fromQuaternion = function(quaternion, result) {
                if (!defined(quaternion)) {
            throw new DeveloperError('quaternion is required');
        }
                if (!defined(result)) {
            result = new HeadingPitchRoll();
        }
        var test = 2 * (quaternion.w * quaternion.y - quaternion.z * quaternion.x);
        var denominatorRoll = 1 - 2 * (quaternion.x * quaternion.x + quaternion.y * quaternion.y);
        var numeratorRoll = 2 * (quaternion.w * quaternion.x + quaternion.y * quaternion.z);
        var denominatorHeading = 1 - 2 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z);
        var numeratorHeading = 2 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y);
        result.heading = -Math.atan2(numeratorHeading, denominatorHeading);
        result.roll = Math.atan2(numeratorRoll, denominatorRoll);
        result.pitch = -CesiumMath.asinClamped(test);
        return result;
    };

    /**
     * Returns a new HeadingPitchRoll instance from angles given in degrees.
     *
     * @param {Number} heading the heading in degrees
     * @param {Number} pitch the pitch in degrees
     * @param {Number} roll the heading in degrees
     * @param {HeadingPitchRoll} [result] The object in which to store the result. If not provided, a new instance is created and returned.
     * @returns {HeadingPitchRoll} A new HeadingPitchRoll instance
     */
    HeadingPitchRoll.fromDegrees = function(heading, pitch, roll, result) {
                if (!defined(heading)) {
            throw new DeveloperError('heading is required');
        }
        if (!defined(pitch)) {
            throw new DeveloperError('pitch is required');
        }
        if (!defined(roll)) {
            throw new DeveloperError('roll is required');
        }
                if (!defined(result)) {
            result = new HeadingPitchRoll();
        }
        result.heading = heading * CesiumMath.RADIANS_PER_DEGREE;
        result.pitch = pitch * CesiumMath.RADIANS_PER_DEGREE;
        result.roll = roll * CesiumMath.RADIANS_PER_DEGREE;
        return result;
    };

    /**
     * Duplicates a HeadingPitchRoll instance.
     *
     * @param {HeadingPitchRoll} headingPitchRoll The HeadingPitchRoll to duplicate.
     * @param {HeadingPitchRoll} [result] The object onto which to store the result.
     * @returns {HeadingPitchRoll} The modified result parameter or a new HeadingPitchRoll instance if one was not provided. (Returns undefined if headingPitchRoll is undefined)
     */
    HeadingPitchRoll.clone = function(headingPitchRoll, result) {
        if (!defined(headingPitchRoll)) {
            return undefined;
        }
        if (!defined(result)) {
            return new HeadingPitchRoll(headingPitchRoll.heading, headingPitchRoll.pitch, headingPitchRoll.roll);
        }
        result.heading = headingPitchRoll.heading;
        result.pitch = headingPitchRoll.pitch;
        result.roll = headingPitchRoll.roll;
        return result;
    };

    /**
     * Compares the provided HeadingPitchRolls componentwise and returns
     * <code>true</code> if they are equal, <code>false</code> otherwise.
     *
     * @param {HeadingPitchRoll} [left] The first HeadingPitchRoll.
     * @param {HeadingPitchRoll} [right] The second HeadingPitchRoll.
     * @returns {Boolean} <code>true</code> if left and right are equal, <code>false</code> otherwise.
     */
    HeadingPitchRoll.equals = function(left, right) {
        return (left === right) ||
            ((defined(left)) &&
                (defined(right)) &&
                (left.heading === right.heading) &&
                (left.pitch === right.pitch) &&
                (left.roll === right.roll));
    };

    /**
     * Compares the provided HeadingPitchRolls componentwise and returns
     * <code>true</code> if they pass an absolute or relative tolerance test,
     * <code>false</code> otherwise.
     *
     * @param {HeadingPitchRoll} [left] The first HeadingPitchRoll.
     * @param {HeadingPitchRoll} [right] The second HeadingPitchRoll.
     * @param {Number} relativeEpsilon The relative epsilon tolerance to use for equality testing.
     * @param {Number} [absoluteEpsilon=relativeEpsilon] The absolute epsilon tolerance to use for equality testing.
     * @returns {Boolean} <code>true</code> if left and right are within the provided epsilon, <code>false</code> otherwise.
     */
    HeadingPitchRoll.equalsEpsilon = function(left, right, relativeEpsilon, absoluteEpsilon) {
        return (left === right) ||
            (defined(left) &&
                defined(right) &&
                CesiumMath.equalsEpsilon(left.heading, right.heading, relativeEpsilon, absoluteEpsilon) &&
                CesiumMath.equalsEpsilon(left.pitch, right.pitch, relativeEpsilon, absoluteEpsilon) &&
                CesiumMath.equalsEpsilon(left.roll, right.roll, relativeEpsilon, absoluteEpsilon));
    };

    /**
     * Duplicates this HeadingPitchRoll instance.
     *
     * @param {HeadingPitchRoll} [result] The object onto which to store the result.
     * @returns {HeadingPitchRoll} The modified result parameter or a new HeadingPitchRoll instance if one was not provided.
     */
    HeadingPitchRoll.prototype.clone = function(result) {
        return HeadingPitchRoll.clone(this, result);
    };

    /**
     * Compares this HeadingPitchRoll against the provided HeadingPitchRoll componentwise and returns
     * <code>true</code> if they are equal, <code>false</code> otherwise.
     *
     * @param {HeadingPitchRoll} [right] The right hand side HeadingPitchRoll.
     * @returns {Boolean} <code>true</code> if they are equal, <code>false</code> otherwise.
     */
    HeadingPitchRoll.prototype.equals = function(right) {
        return HeadingPitchRoll.equals(this, right);
    };

    /**
     * Compares this HeadingPitchRoll against the provided HeadingPitchRoll componentwise and returns
     * <code>true</code> if they pass an absolute or relative tolerance test,
     * <code>false</code> otherwise.
     *
     * @param {HeadingPitchRoll} [right] The right hand side HeadingPitchRoll.
     * @param {Number} relativeEpsilon The relative epsilon tolerance to use for equality testing.
     * @param {Number} [absoluteEpsilon=relativeEpsilon] The absolute epsilon tolerance to use for equality testing.
     * @returns {Boolean} <code>true</code> if they are within the provided epsilon, <code>false</code> otherwise.
     */
    HeadingPitchRoll.prototype.equalsEpsilon = function(right, relativeEpsilon, absoluteEpsilon) {
        return HeadingPitchRoll.equalsEpsilon(this, right, relativeEpsilon, absoluteEpsilon);
    };

    /**
     * Creates a string representing this HeadingPitchRoll in the format '(heading, pitch, roll)' in radians.
     *
     * @returns {String} A string representing the provided HeadingPitchRoll in the format '(heading, pitch, roll)'.
     */
    HeadingPitchRoll.prototype.toString = function() {
        return '(' + this.heading + ', ' + this.pitch + ', ' + this.roll + ')';
    };

    return HeadingPitchRoll;
});

define('Core/buildModuleUrl',[
        './defined',
        './DeveloperError',
        './getAbsoluteUri',
        './Resource',
        'require'
    ], function(
        defined,
        DeveloperError,
        getAbsoluteUri,
        Resource,
        require) {
    'use strict';
    /*global CESIUM_BASE_URL*/

    var cesiumScriptRegex = /((?:.*\/)|^)cesium[\w-]*\.js(?:\W|$)/i;
    function getBaseUrlFromCesiumScript() {
        var scripts = document.getElementsByTagName('script');
        for ( var i = 0, len = scripts.length; i < len; ++i) {
            var src = scripts[i].getAttribute('src');
            var result = cesiumScriptRegex.exec(src);
            if (result !== null) {
                return result[1];
            }
        }
        return undefined;
    }

    var a;
    function tryMakeAbsolute(url) {
        if (typeof document === 'undefined') {
            //Node.js and Web Workers. In both cases, the URL will already be absolute.
            return url;
        }

        if (!defined(a)) {
            a = document.createElement('a');
        }
        a.href = url;

        // IE only absolutizes href on get, not set
        a.href = a.href; // eslint-disable-line no-self-assign
        return a.href;
    }

    var baseResource;
    function getCesiumBaseUrl() {
        if (defined(baseResource)) {
            return baseResource;
        }

        var baseUrlString;
        if (typeof CESIUM_BASE_URL !== 'undefined') {
            baseUrlString = CESIUM_BASE_URL;
        } else if (defined(define.amd) && !define.amd.toUrlUndefined && defined(require.toUrl)) {
            baseUrlString = getAbsoluteUri('..', buildModuleUrl('Core/buildModuleUrl.js'));
        } else {
            baseUrlString = getBaseUrlFromCesiumScript();
        }

                if (!defined(baseUrlString)) {
            throw new DeveloperError('Unable to determine Cesium base URL automatically, try defining a global variable called CESIUM_BASE_URL.');
        }
        
        baseResource = new Resource({
            url: tryMakeAbsolute(baseUrlString)
        });
        baseResource.appendForwardSlash();

        return baseResource;
    }

    function buildModuleUrlFromRequireToUrl(moduleID) {
        //moduleID will be non-relative, so require it relative to this module, in Core.
        return tryMakeAbsolute(require.toUrl('../' + moduleID));
    }

    function buildModuleUrlFromBaseUrl(moduleID) {
        var resource = getCesiumBaseUrl().getDerivedResource({
            url: moduleID
        });
        return resource.url;
    }

    var implementation;

    /**
     * Given a non-relative moduleID, returns an absolute URL to the file represented by that module ID,
     * using, in order of preference, require.toUrl, the value of a global CESIUM_BASE_URL, or
     * the base URL of the Cesium.js script.
     *
     * @private
     */
    function buildModuleUrl(moduleID) {
        if (!defined(implementation)) {
            //select implementation
            if (defined(define.amd) && !define.amd.toUrlUndefined && defined(require.toUrl)) {
                implementation = buildModuleUrlFromRequireToUrl;
            } else {
                implementation = buildModuleUrlFromBaseUrl;
            }
        }

        var url = implementation(moduleID);
        return url;
    }

    // exposed for testing
    buildModuleUrl._cesiumScriptRegex = cesiumScriptRegex;
    buildModuleUrl._buildModuleUrlFromBaseUrl = buildModuleUrlFromBaseUrl;
    buildModuleUrl._clearBaseResource = function() {
        baseResource = undefined;
    };

    /**
     * Sets the base URL for resolving modules.
     * @param {String} value The new base URL.
     */
    buildModuleUrl.setBaseUrl = function(value) {
        baseResource = Resource.DEFAULT.getDerivedResource({
            url: value
        });
    };

    /**
     * Gets the base URL for resolving modules.
     */
    buildModuleUrl.getCesiumBaseUrl = getCesiumBaseUrl;

    return buildModuleUrl;
});

define('Core/Iau2006XysSample',[],function() {
    'use strict';

    /**
     * An IAU 2006 XYS value sampled at a particular time.
     *
     * @alias Iau2006XysSample
     * @constructor
     *
     * @param {Number} x The X value.
     * @param {Number} y The Y value.
     * @param {Number} s The S value.
     *
     * @private
     */
    function Iau2006XysSample(x, y, s) {
        /**
         * The X value.
         * @type {Number}
         */
        this.x = x;

        /**
         * The Y value.
         * @type {Number}
         */
        this.y = y;

        /**
         * The S value.
         * @type {Number}
         */
        this.s = s;
    }

    return Iau2006XysSample;
});

define('Core/Iau2006XysData',[
        '../ThirdParty/when',
        './buildModuleUrl',
        './defaultValue',
        './defined',
        './Iau2006XysSample',
        './JulianDate',
        './Resource',
        './TimeStandard'
    ], function(
        when,
        buildModuleUrl,
        defaultValue,
        defined,
        Iau2006XysSample,
        JulianDate,
        Resource,
        TimeStandard) {
    'use strict';

    /**
     * A set of IAU2006 XYS data that is used to evaluate the transformation between the International
     * Celestial Reference Frame (ICRF) and the International Terrestrial Reference Frame (ITRF).
     *
     * @alias Iau2006XysData
     * @constructor
     *
     * @param {Object} [options] Object with the following properties:
     * @param {Resource|String} [options.xysFileUrlTemplate='Assets/IAU2006_XYS/IAU2006_XYS_{0}.json'] A template URL for obtaining the XYS data.  In the template,
     *                 `{0}` will be replaced with the file index.
     * @param {Number} [options.interpolationOrder=9] The order of interpolation to perform on the XYS data.
     * @param {Number} [options.sampleZeroJulianEphemerisDate=2442396.5] The Julian ephemeris date (JED) of the
     *                 first XYS sample.
     * @param {Number} [options.stepSizeDays=1.0] The step size, in days, between successive XYS samples.
     * @param {Number} [options.samplesPerXysFile=1000] The number of samples in each XYS file.
     * @param {Number} [options.totalSamples=27426] The total number of samples in all XYS files.
     *
     * @private
     */
    function Iau2006XysData(options) {
        options = defaultValue(options, defaultValue.EMPTY_OBJECT);

        this._xysFileUrlTemplate = Resource.createIfNeeded(options.xysFileUrlTemplate);
        this._interpolationOrder = defaultValue(options.interpolationOrder, 9);
        this._sampleZeroJulianEphemerisDate = defaultValue(options.sampleZeroJulianEphemerisDate, 2442396.5);
        this._sampleZeroDateTT = new JulianDate(this._sampleZeroJulianEphemerisDate, 0.0, TimeStandard.TAI);
        this._stepSizeDays = defaultValue(options.stepSizeDays, 1.0);
        this._samplesPerXysFile = defaultValue(options.samplesPerXysFile, 1000);
        this._totalSamples = defaultValue(options.totalSamples, 27426);
        this._samples = new Array(this._totalSamples * 3);
        this._chunkDownloadsInProgress = [];

        var order = this._interpolationOrder;

        // Compute denominators and X values for interpolation.
        var denom = this._denominators = new Array(order + 1);
        var xTable = this._xTable = new Array(order + 1);

        var stepN = Math.pow(this._stepSizeDays, order);

        for ( var i = 0; i <= order; ++i) {
            denom[i] = stepN;
            xTable[i] = i * this._stepSizeDays;

            for ( var j = 0; j <= order; ++j) {
                if (j !== i) {
                    denom[i] *= (i - j);
                }
            }

            denom[i] = 1.0 / denom[i];
        }

        // Allocate scratch arrays for interpolation.
        this._work = new Array(order + 1);
        this._coef = new Array(order + 1);
    }

    var julianDateScratch = new JulianDate(0, 0.0, TimeStandard.TAI);

    function getDaysSinceEpoch(xys, dayTT, secondTT) {
        var dateTT = julianDateScratch;
        dateTT.dayNumber = dayTT;
        dateTT.secondsOfDay = secondTT;
        return JulianDate.daysDifference(dateTT, xys._sampleZeroDateTT);
    }

    /**
     * Preloads XYS data for a specified date range.
     *
     * @param {Number} startDayTT The Julian day number of the beginning of the interval to preload, expressed in
     *                 the Terrestrial Time (TT) time standard.
     * @param {Number} startSecondTT The seconds past noon of the beginning of the interval to preload, expressed in
     *                 the Terrestrial Time (TT) time standard.
     * @param {Number} stopDayTT The Julian day number of the end of the interval to preload, expressed in
     *                 the Terrestrial Time (TT) time standard.
     * @param {Number} stopSecondTT The seconds past noon of the end of the interval to preload, expressed in
     *                 the Terrestrial Time (TT) time standard.
     * @returns {Promise} A promise that, when resolved, indicates that the requested interval has been
     *                    preloaded.
     */
    Iau2006XysData.prototype.preload = function(startDayTT, startSecondTT, stopDayTT, stopSecondTT) {
        var startDaysSinceEpoch = getDaysSinceEpoch(this, startDayTT, startSecondTT);
        var stopDaysSinceEpoch = getDaysSinceEpoch(this, stopDayTT, stopSecondTT);

        var startIndex = (startDaysSinceEpoch / this._stepSizeDays - this._interpolationOrder / 2) | 0;
        if (startIndex < 0) {
            startIndex = 0;
        }

        var stopIndex = (stopDaysSinceEpoch / this._stepSizeDays - this._interpolationOrder / 2) | 0 + this._interpolationOrder;
        if (stopIndex >= this._totalSamples) {
            stopIndex = this._totalSamples - 1;
        }

        var startChunk = (startIndex / this._samplesPerXysFile) | 0;
        var stopChunk = (stopIndex / this._samplesPerXysFile) | 0;

        var promises = [];
        for ( var i = startChunk; i <= stopChunk; ++i) {
            promises.push(requestXysChunk(this, i));
        }

        return when.all(promises);
    };

    /**
     * Computes the XYS values for a given date by interpolating.  If the required data is not yet downloaded,
     * this method will return undefined.
     *
     * @param {Number} dayTT The Julian day number for which to compute the XYS value, expressed in
     *                 the Terrestrial Time (TT) time standard.
     * @param {Number} secondTT The seconds past noon of the date for which to compute the XYS value, expressed in
     *                 the Terrestrial Time (TT) time standard.
     * @param {Iau2006XysSample} [result] The instance to which to copy the interpolated result.  If this parameter
     *                           is undefined, a new instance is allocated and returned.
     * @returns {Iau2006XysSample} The interpolated XYS values, or undefined if the required data for this
     *                             computation has not yet been downloaded.
     *
     * @see Iau2006XysData#preload
     */
    Iau2006XysData.prototype.computeXysRadians = function(dayTT, secondTT, result) {
        var daysSinceEpoch = getDaysSinceEpoch(this, dayTT, secondTT);
        if (daysSinceEpoch < 0.0) {
            // Can't evaluate prior to the epoch of the data.
            return undefined;
        }

        var centerIndex = (daysSinceEpoch / this._stepSizeDays) | 0;
        if (centerIndex >= this._totalSamples) {
            // Can't evaluate after the last sample in the data.
            return undefined;
        }

        var degree = this._interpolationOrder;

        var firstIndex = centerIndex - ((degree / 2) | 0);
        if (firstIndex < 0) {
            firstIndex = 0;
        }
        var lastIndex = firstIndex + degree;
        if (lastIndex >= this._totalSamples) {
            lastIndex = this._totalSamples - 1;
            firstIndex = lastIndex - degree;
            if (firstIndex < 0) {
                firstIndex = 0;
            }
        }

        // Are all the samples we need present?
        // We can assume so if the first and last are present
        var isDataMissing = false;
        var samples = this._samples;
        if (!defined(samples[firstIndex * 3])) {
            requestXysChunk(this, (firstIndex / this._samplesPerXysFile) | 0);
            isDataMissing = true;
        }

        if (!defined(samples[lastIndex * 3])) {
            requestXysChunk(this, (lastIndex / this._samplesPerXysFile) | 0);
            isDataMissing = true;
        }

        if (isDataMissing) {
            return undefined;
        }

        if (!defined(result)) {
            result = new Iau2006XysSample(0.0, 0.0, 0.0);
        } else {
            result.x = 0.0;
            result.y = 0.0;
            result.s = 0.0;
        }

        var x = daysSinceEpoch - firstIndex * this._stepSizeDays;

        var work = this._work;
        var denom = this._denominators;
        var coef = this._coef;
        var xTable = this._xTable;

        var i, j;
        for (i = 0; i <= degree; ++i) {
            work[i] = x - xTable[i];
        }

        for (i = 0; i <= degree; ++i) {
            coef[i] = 1.0;

            for (j = 0; j <= degree; ++j) {
                if (j !== i) {
                    coef[i] *= work[j];
                }
            }

            coef[i] *= denom[i];

            var sampleIndex = (firstIndex + i) * 3;
            result.x += coef[i] * samples[sampleIndex++];
            result.y += coef[i] * samples[sampleIndex++];
            result.s += coef[i] * samples[sampleIndex];
        }

        return result;
    };

    function requestXysChunk(xysData, chunkIndex) {
        if (xysData._chunkDownloadsInProgress[chunkIndex]) {
            // Chunk has already been requested.
            return xysData._chunkDownloadsInProgress[chunkIndex];
        }

        var deferred = when.defer();

        xysData._chunkDownloadsInProgress[chunkIndex] = deferred;

        var chunkUrl;
        var xysFileUrlTemplate = xysData._xysFileUrlTemplate;
        if (defined(xysFileUrlTemplate)) {
            chunkUrl = xysFileUrlTemplate.getDerivedResource({
                templateValues: {
                    '0': chunkIndex
                }
            });
        } else {
            chunkUrl = new Resource({
                url : buildModuleUrl('Assets/IAU2006_XYS/IAU2006_XYS_' + chunkIndex + '.json')
            });
        }

        when(chunkUrl.fetchJson(), function(chunk) {
            xysData._chunkDownloadsInProgress[chunkIndex] = false;

            var samples = xysData._samples;
            var newSamples = chunk.samples;
            var startIndex = chunkIndex * xysData._samplesPerXysFile * 3;

            for ( var i = 0, len = newSamples.length; i < len; ++i) {
                samples[startIndex + i] = newSamples[i];
            }

            deferred.resolve();
        });

        return deferred.promise;
    }

    return Iau2006XysData;
});

define('Core/Transforms',[
        '../ThirdParty/when',
        './Cartesian2',
        './Cartesian3',
        './Cartesian4',
        './Cartographic',
        './Check',
        './defaultValue',
        './defined',
        './DeveloperError',
        './EarthOrientationParameters',
        './EarthOrientationParametersSample',
        './Ellipsoid',
        './HeadingPitchRoll',
        './Iau2006XysData',
        './Iau2006XysSample',
        './JulianDate',
        './Math',
        './Matrix3',
        './Matrix4',
        './Quaternion',
        './TimeConstants'
    ], function(
        when,
        Cartesian2,
        Cartesian3,
        Cartesian4,
        Cartographic,
        Check,
        defaultValue,
        defined,
        DeveloperError,
        EarthOrientationParameters,
        EarthOrientationParametersSample,
        Ellipsoid,
        HeadingPitchRoll,
        Iau2006XysData,
        Iau2006XysSample,
        JulianDate,
        CesiumMath,
        Matrix3,
        Matrix4,
        Quaternion,
        TimeConstants) {
    'use strict';

    /**
     * Contains functions for transforming positions to various reference frames.
     *
     * @exports Transforms
     * @namespace
     */
    var Transforms = {};

    var vectorProductLocalFrame = {
        up : {
            south : 'east',
            north : 'west',
            west : 'south',
            east : 'north'
        },
        down : {
            south : 'west',
            north : 'east',
            west : 'north',
            east : 'south'
        },
        south : {
            up : 'west',
            down : 'east',
            west : 'down',
            east : 'up'
        },
        north : {
            up : 'east',
            down : 'west',
            west : 'up',
            east : 'down'
        },
        west : {
            up : 'north',
            down : 'south',
            north : 'down',
            south : 'up'
        },
        east : {
            up : 'south',
            down : 'north',
            north : 'up',
            south : 'down'
        }
    };

    var degeneratePositionLocalFrame = {
        north : [-1, 0, 0],
        east : [0, 1, 0],
        up : [0, 0, 1],
        south : [1, 0, 0],
        west : [0, -1, 0],
        down : [0, 0, -1]
    };

    var localFrameToFixedFrameCache = {};

    var scratchCalculateCartesian = {
        east : new Cartesian3(),
        north : new Cartesian3(),
        up : new Cartesian3(),
        west : new Cartesian3(),
        south : new Cartesian3(),
        down : new Cartesian3()
    };
    var scratchFirstCartesian = new Cartesian3();
    var scratchSecondCartesian = new Cartesian3();
    var scratchThirdCartesian = new Cartesian3();
    /**
    * Generates a function that computes a 4x4 transformation matrix from a reference frame
    * centered at the provided origin to the provided ellipsoid's fixed reference frame.
    * @param  {String} firstAxis  name of the first axis of the local reference frame. Must be
    *  'east', 'north', 'up', 'west', 'south' or 'down'.
    * @param  {String} secondAxis  name of the second axis of the local reference frame. Must be
    *  'east', 'north', 'up', 'west', 'south' or 'down'.
    * @return {localFrameToFixedFrameGenerator~resultat} The function that will computes a
    * 4x4 transformation matrix from a reference frame, with first axis and second axis compliant with the parameters,
    */
    Transforms.localFrameToFixedFrameGenerator = function (firstAxis, secondAxis) {
        if (!vectorProductLocalFrame.hasOwnProperty(firstAxis) || !vectorProductLocalFrame[firstAxis].hasOwnProperty(secondAxis)) {
            throw new DeveloperError('firstAxis and secondAxis must be east, north, up, west, south or down.');
        }
        var thirdAxis = vectorProductLocalFrame[firstAxis][secondAxis];

        /**
         * Computes a 4x4 transformation matrix from a reference frame
         * centered at the provided origin to the provided ellipsoid's fixed reference frame.
         * @callback Transforms~LocalFrameToFixedFrame
         * @param {Cartesian3} origin The center point of the local reference frame.
         * @param {Ellipsoid} [ellipsoid=Ellipsoid.WGS84] The ellipsoid whose fixed frame is used in the transformation.
         * @param {Matrix4} [result] The object onto which to store the result.
         * @returns {Matrix4} The modified result parameter or a new Matrix4 instance if none was provided.
         */
        var resultat;
        var hashAxis = firstAxis + secondAxis;
        if (defined(localFrameToFixedFrameCache[hashAxis])) {
            resultat = localFrameToFixedFrameCache[hashAxis];
        } else {
            resultat = function (origin, ellipsoid, result) {
                                if (!defined(origin)) {
                    throw new DeveloperError('origin is required.');
                }
                                if (!defined(result)) {
                    result = new Matrix4();
                }
                // If x and y are zero, assume origin is at a pole, which is a special case.
                if (CesiumMath.equalsEpsilon(origin.x, 0.0, CesiumMath.EPSILON14) && CesiumMath.equalsEpsilon(origin.y, 0.0, CesiumMath.EPSILON14)) {
                    var sign = CesiumMath.sign(origin.z);

                    Cartesian3.unpack(degeneratePositionLocalFrame[firstAxis], 0, scratchFirstCartesian);
                    if (firstAxis !== 'east' && firstAxis !== 'west') {
                        Cartesian3.multiplyByScalar(scratchFirstCartesian, sign, scratchFirstCartesian);
                    }

                    Cartesian3.unpack(degeneratePositionLocalFrame[secondAxis], 0, scratchSecondCartesian);
                    if (secondAxis !== 'east' && secondAxis !== 'west') {
                        Cartesian3.multiplyByScalar(scratchSecondCartesian, sign, scratchSecondCartesian);
                    }

                    Cartesian3.unpack(degeneratePositionLocalFrame[thirdAxis], 0, scratchThirdCartesian);
                    if (thirdAxis !== 'east' && thirdAxis !== 'west') {
                        Cartesian3.multiplyByScalar(scratchThirdCartesian, sign, scratchThirdCartesian);
                    }
                } else {
                    ellipsoid = defaultValue(ellipsoid, Ellipsoid.WGS84);
                    ellipsoid.geodeticSurfaceNormal(origin, scratchCalculateCartesian.up);

                    var up = scratchCalculateCartesian.up;
                    var east = scratchCalculateCartesian.east;
                    east.x = -origin.y;
                    east.y = origin.x;
                    east.z = 0.0;
                    Cartesian3.normalize(east, scratchCalculateCartesian.east);
                    Cartesian3.cross(up, east, scratchCalculateCartesian.north);

                    Cartesian3.multiplyByScalar(scratchCalculateCartesian.up, -1, scratchCalculateCartesian.down);
                    Cartesian3.multiplyByScalar(scratchCalculateCartesian.east, -1, scratchCalculateCartesian.west);
                    Cartesian3.multiplyByScalar(scratchCalculateCartesian.north, -1, scratchCalculateCartesian.south);

                    scratchFirstCartesian = scratchCalculateCartesian[firstAxis];
                    scratchSecondCartesian = scratchCalculateCartesian[secondAxis];
                    scratchThirdCartesian = scratchCalculateCartesian[thirdAxis];
                }
                result[0] = scratchFirstCartesian.x;
                result[1] = scratchFirstCartesian.y;
                result[2] = scratchFirstCartesian.z;
                result[3] = 0.0;
                result[4] = scratchSecondCartesian.x;
                result[5] = scratchSecondCartesian.y;
                result[6] = scratchSecondCartesian.z;
                result[7] = 0.0;
                result[8] = scratchThirdCartesian.x;
                result[9] = scratchThirdCartesian.y;
                result[10] = scratchThirdCartesian.z;
                result[11] = 0.0;
                result[12] = origin.x;
                result[13] = origin.y;
                result[14] = origin.z;
                result[15] = 1.0;
                return result;
            };
            localFrameToFixedFrameCache[hashAxis] = resultat;
        }
        return resultat;
    };

    /**
     * Computes a 4x4 transformation matrix from a reference frame with an east-north-up axes
     * centered at the provided origin to the provided ellipsoid's fixed reference frame.
     * The local axes are defined as:
     * <ul>
     * <li>The <code>x</code> axis points in the local east direction.</li>
     * <li>The <code>y</code> axis points in the local north direction.</li>
     * <li>The <code>z</code> axis points in the direction of the ellipsoid surface normal which passes through the position.</li>
     * </ul>
     *
     * @function
     * @param {Cartesian3} origin The center point of the local reference frame.
     * @param {Ellipsoid} [ellipsoid=Ellipsoid.WGS84] The ellipsoid whose fixed frame is used in the transformation.
     * @param {Matrix4} [result] The object onto which to store the result.
     * @returns {Matrix4} The modified result parameter or a new Matrix4 instance if none was provided.
     *
     * @example
     * // Get the transform from local east-north-up at cartographic (0.0, 0.0) to Earth's fixed frame.
     * var center = Cesium.Cartesian3.fromDegrees(0.0, 0.0);
     * var transform = Cesium.Transforms.eastNorthUpToFixedFrame(center);
     */
    Transforms.eastNorthUpToFixedFrame = Transforms.localFrameToFixedFrameGenerator('east','north');

    /**
     * Computes a 4x4 transformation matrix from a reference frame with an north-east-down axes
     * centered at the provided origin to the provided ellipsoid's fixed reference frame.
     * The local axes are defined as:
     * <ul>
     * <li>The <code>x</code> axis points in the local north direction.</li>
     * <li>The <code>y</code> axis points in the local east direction.</li>
     * <li>The <code>z</code> axis points in the opposite direction of the ellipsoid surface normal which passes through the position.</li>
     * </ul>
     *
     * @function
     * @param {Cartesian3} origin The center point of the local reference frame.
     * @param {Ellipsoid} [ellipsoid=Ellipsoid.WGS84] The ellipsoid whose fixed frame is used in the transformation.
     * @param {Matrix4} [result] The object onto which to store the result.
     * @returns {Matrix4} The modified result parameter or a new Matrix4 instance if none was provided.
     *
     * @example
     * // Get the transform from local north-east-down at cartographic (0.0, 0.0) to Earth's fixed frame.
     * var center = Cesium.Cartesian3.fromDegrees(0.0, 0.0);
     * var transform = Cesium.Transforms.northEastDownToFixedFrame(center);
     */
    Transforms.northEastDownToFixedFrame = Transforms.localFrameToFixedFrameGenerator('north','east');

    /**
     * Computes a 4x4 transformation matrix from a reference frame with an north-up-east axes
     * centered at the provided origin to the provided ellipsoid's fixed reference frame.
     * The local axes are defined as:
     * <ul>
     * <li>The <code>x</code> axis points in the local north direction.</li>
     * <li>The <code>y</code> axis points in the direction of the ellipsoid surface normal which passes through the position.</li>
     * <li>The <code>z</code> axis points in the local east direction.</li>
     * </ul>
     *
     * @function
     * @param {Cartesian3} origin The center point of the local reference frame.
     * @param {Ellipsoid} [ellipsoid=Ellipsoid.WGS84] The ellipsoid whose fixed frame is used in the transformation.
     * @param {Matrix4} [result] The object onto which to store the result.
     * @returns {Matrix4} The modified result parameter or a new Matrix4 instance if none was provided.
     *
     * @example
     * // Get the transform from local north-up-east at cartographic (0.0, 0.0) to Earth's fixed frame.
     * var center = Cesium.Cartesian3.fromDegrees(0.0, 0.0);
     * var transform = Cesium.Transforms.northUpEastToFixedFrame(center);
     */
    Transforms.northUpEastToFixedFrame = Transforms.localFrameToFixedFrameGenerator('north','up');

    /**
     * Computes a 4x4 transformation matrix from a reference frame with an north-west-up axes
     * centered at the provided origin to the provided ellipsoid's fixed reference frame.
     * The local axes are defined as:
     * <ul>
     * <li>The <code>x</code> axis points in the local north direction.</li>
     * <li>The <code>y</code> axis points in the local west direction.</li>
     * <li>The <code>z</code> axis points in the direction of the ellipsoid surface normal which passes through the position.</li>
     * </ul>
     *
     * @function
     * @param {Cartesian3} origin The center point of the local reference frame.
     * @param {Ellipsoid} [ellipsoid=Ellipsoid.WGS84] The ellipsoid whose fixed frame is used in the transformation.
     * @param {Matrix4} [result] The object onto which to store the result.
     * @returns {Matrix4} The modified result parameter or a new Matrix4 instance if none was provided.
     *
      * @example
     * // Get the transform from local north-West-Up at cartographic (0.0, 0.0) to Earth's fixed frame.
     * var center = Cesium.Cartesian3.fromDegrees(0.0, 0.0);
     * var transform = Cesium.Transforms.northWestUpToFixedFrame(center);
     */
    Transforms.northWestUpToFixedFrame = Transforms.localFrameToFixedFrameGenerator('north','west');

    var scratchHPRQuaternion = new Quaternion();
    var scratchScale = new Cartesian3(1.0, 1.0, 1.0);
    var scratchHPRMatrix4 = new Matrix4();

    /**
     * Computes a 4x4 transformation matrix from a reference frame with axes computed from the heading-pitch-roll angles
     * centered at the provided origin to the provided ellipsoid's fixed reference frame. Heading is the rotation from the local north
     * direction where a positive angle is increasing eastward. Pitch is the rotation from the local east-north plane. Positive pitch angles
     * are above the plane. Negative pitch angles are below the plane. Roll is the first rotation applied about the local east axis.
     *
     * @param {Cartesian3} origin The center point of the local reference frame.
     * @param {HeadingPitchRoll} headingPitchRoll The heading, pitch, and roll.
     * @param {Ellipsoid} [ellipsoid=Ellipsoid.WGS84] The ellipsoid whose fixed frame is used in the transformation.
     * @param {Transforms~LocalFrameToFixedFrame} [fixedFrameTransform=Transforms.eastNorthUpToFixedFrame] A 4x4 transformation
     *  matrix from a reference frame to the provided ellipsoid's fixed reference frame
     * @param {Matrix4} [result] The object onto which to store the result.
     * @returns {Matrix4} The modified result parameter or a new Matrix4 instance if none was provided.
     *
     * @example
     * // Get the transform from local heading-pitch-roll at cartographic (0.0, 0.0) to Earth's fixed frame.
     * var center = Cesium.Cartesian3.fromDegrees(0.0, 0.0);
     * var heading = -Cesium.Math.PI_OVER_TWO;
     * var pitch = Cesium.Math.PI_OVER_FOUR;
     * var roll = 0.0;
     * var hpr = new Cesium.HeadingPitchRoll(heading, pitch, roll);
     * var transform = Cesium.Transforms.headingPitchRollToFixedFrame(center, hpr);
     */
    Transforms.headingPitchRollToFixedFrame = function(origin, headingPitchRoll, ellipsoid, fixedFrameTransform, result) {
                Check.typeOf.object( 'HeadingPitchRoll', headingPitchRoll);
        
        fixedFrameTransform = defaultValue(fixedFrameTransform, Transforms.eastNorthUpToFixedFrame);
        var hprQuaternion = Quaternion.fromHeadingPitchRoll(headingPitchRoll, scratchHPRQuaternion);
        var hprMatrix = Matrix4.fromTranslationQuaternionRotationScale(Cartesian3.ZERO, hprQuaternion, scratchScale, scratchHPRMatrix4);
        result = fixedFrameTransform(origin, ellipsoid, result);
        return Matrix4.multiply(result, hprMatrix, result);
    };

    var scratchENUMatrix4 = new Matrix4();
    var scratchHPRMatrix3 = new Matrix3();

    /**
     * Computes a quaternion from a reference frame with axes computed from the heading-pitch-roll angles
     * centered at the provided origin. Heading is the rotation from the local north
     * direction where a positive angle is increasing eastward. Pitch is the rotation from the local east-north plane. Positive pitch angles
     * are above the plane. Negative pitch angles are below the plane. Roll is the first rotation applied about the local east axis.
     *
     * @param {Cartesian3} origin The center point of the local reference frame.
     * @param {HeadingPitchRoll} headingPitchRoll The heading, pitch, and roll.
     * @param {Ellipsoid} [ellipsoid=Ellipsoid.WGS84] The ellipsoid whose fixed frame is used in the transformation.
     * @param {Transforms~LocalFrameToFixedFrame} [fixedFrameTransform=Transforms.eastNorthUpToFixedFrame] A 4x4 transformation
     *  matrix from a reference frame to the provided ellipsoid's fixed reference frame
     * @param {Quaternion} [result] The object onto which to store the result.
     * @returns {Quaternion} The modified result parameter or a new Quaternion instance if none was provided.
     *
     * @example
     * // Get the quaternion from local heading-pitch-roll at cartographic (0.0, 0.0) to Earth's fixed frame.
     * var center = Cesium.Cartesian3.fromDegrees(0.0, 0.0);
     * var heading = -Cesium.Math.PI_OVER_TWO;
     * var pitch = Cesium.Math.PI_OVER_FOUR;
     * var roll = 0.0;
     * var hpr = new HeadingPitchRoll(heading, pitch, roll);
     * var quaternion = Cesium.Transforms.headingPitchRollQuaternion(center, hpr);
     */
    Transforms.headingPitchRollQuaternion = function(origin, headingPitchRoll, ellipsoid, fixedFrameTransform, result) {
                Check.typeOf.object( 'HeadingPitchRoll', headingPitchRoll);
        
        var transform = Transforms.headingPitchRollToFixedFrame(origin, headingPitchRoll, ellipsoid, fixedFrameTransform, scratchENUMatrix4);
        var rotation = Matrix4.getRotation(transform, scratchHPRMatrix3);
        return Quaternion.fromRotationMatrix(rotation, result);
    };

    var noScale = new Cartesian3(1.0, 1.0, 1.0);
    var hprCenterScratch = new Cartesian3();
    var ffScratch = new Matrix4();
    var hprTransformScratch = new Matrix4();
    var hprRotationScratch = new Matrix3();
    var hprQuaternionScratch = new Quaternion();
    /**
     * Computes heading-pitch-roll angles from a transform in a particular reference frame. Heading is the rotation from the local north
     * direction where a positive angle is increasing eastward. Pitch is the rotation from the local east-north plane. Positive pitch angles
     * are above the plane. Negative pitch angles are below the plane. Roll is the first rotation applied about the local east axis.
     *
     * @param {Matrix4} transform The transform
     * @param {Ellipsoid} [ellipsoid=Ellipsoid.WGS84] The ellipsoid whose fixed frame is used in the transformation.
     * @param {Transforms~LocalFrameToFixedFrame} [fixedFrameTransform=Transforms.eastNorthUpToFixedFrame] A 4x4 transformation
     *  matrix from a reference frame to the provided ellipsoid's fixed reference frame
     * @param {HeadingPitchRoll} [result] The object onto which to store the result.
     * @returns {HeadingPitchRoll} The modified result parameter or a new HeadingPitchRoll instance if none was provided.
     */
    Transforms.fixedFrameToHeadingPitchRoll = function(transform, ellipsoid, fixedFrameTransform, result) {
                Check.defined('transform', transform);
        
        ellipsoid = defaultValue(ellipsoid, Ellipsoid.WGS84);
        fixedFrameTransform = defaultValue(fixedFrameTransform, Transforms.eastNorthUpToFixedFrame);
        if (!defined(result)) {
            result = new HeadingPitchRoll();
        }

        var center = Matrix4.getTranslation(transform, hprCenterScratch);
        if (Cartesian3.equals(center, Cartesian3.ZERO)) {
            result.heading = 0;
            result.pitch = 0;
            result.roll = 0;
            return result;
        }
        var toFixedFrame = Matrix4.inverseTransformation(fixedFrameTransform(center, ellipsoid, ffScratch), ffScratch);
        var transformCopy = Matrix4.setScale(transform, noScale, hprTransformScratch);
        transformCopy = Matrix4.setTranslation(transformCopy, Cartesian3.ZERO, transformCopy);

        toFixedFrame = Matrix4.multiply(toFixedFrame, transformCopy, toFixedFrame);
        var quaternionRotation = Quaternion.fromRotationMatrix(Matrix4.getRotation(toFixedFrame, hprRotationScratch), hprQuaternionScratch);
        quaternionRotation = Quaternion.normalize(quaternionRotation, quaternionRotation);

        return HeadingPitchRoll.fromQuaternion(quaternionRotation, result);
    };

    var gmstConstant0 = 6 * 3600 + 41 * 60 + 50.54841;
    var gmstConstant1 = 8640184.812866;
    var gmstConstant2 = 0.093104;
    var gmstConstant3 = -6.2E-6;
    var rateCoef = 1.1772758384668e-19;
    var wgs84WRPrecessing = 7.2921158553E-5;
    var twoPiOverSecondsInDay = CesiumMath.TWO_PI / 86400.0;
    var dateInUtc = new JulianDate();

    /**
     * Computes a rotation matrix to transform a point or vector from True Equator Mean Equinox (TEME) axes to the
     * pseudo-fixed axes at a given time.  This method treats the UT1 time standard as equivalent to UTC.
     *
     * @param {JulianDate} date The time at which to compute the rotation matrix.
     * @param {Matrix3} [result] The object onto which to store the result.
     * @returns {Matrix3} The modified result parameter or a new Matrix3 instance if none was provided.
     *
     * @example
     * //Set the view to the inertial frame.
     * scene.postUpdate.addEventListener(function(scene, time) {
     *    var now = Cesium.JulianDate.now();
     *    var offset = Cesium.Matrix4.multiplyByPoint(camera.transform, camera.position, new Cesium.Cartesian3());
     *    var transform = Cesium.Matrix4.fromRotationTranslation(Cesium.Transforms.computeTemeToPseudoFixedMatrix(now));
     *    var inverseTransform = Cesium.Matrix4.inverseTransformation(transform, new Cesium.Matrix4());
     *    Cesium.Matrix4.multiplyByPoint(inverseTransform, offset, offset);
     *    camera.lookAtTransform(transform, offset);
     * });
     */
    Transforms.computeTemeToPseudoFixedMatrix = function (date, result) {
                if (!defined(date)) {
            throw new DeveloperError('date is required.');
        }
        
        // GMST is actually computed using UT1.  We're using UTC as an approximation of UT1.
        // We do not want to use the function like convertTaiToUtc in JulianDate because
        // we explicitly do not want to fail when inside the leap second.

        dateInUtc = JulianDate.addSeconds(date, -JulianDate.computeTaiMinusUtc(date), dateInUtc);
        var utcDayNumber = dateInUtc.dayNumber;
        var utcSecondsIntoDay = dateInUtc.secondsOfDay;

        var t;
        var diffDays = utcDayNumber - 2451545;
        if (utcSecondsIntoDay >= 43200.0) {
            t = (diffDays + 0.5) / TimeConstants.DAYS_PER_JULIAN_CENTURY;
        } else {
            t = (diffDays - 0.5) / TimeConstants.DAYS_PER_JULIAN_CENTURY;
        }

        var gmst0 = gmstConstant0 + t * (gmstConstant1 + t * (gmstConstant2 + t * gmstConstant3));
        var angle = (gmst0 * twoPiOverSecondsInDay) % CesiumMath.TWO_PI;
        var ratio = wgs84WRPrecessing + rateCoef * (utcDayNumber - 2451545.5);
        var secondsSinceMidnight = (utcSecondsIntoDay + TimeConstants.SECONDS_PER_DAY * 0.5) % TimeConstants.SECONDS_PER_DAY;
        var gha = angle + (ratio * secondsSinceMidnight);
        var cosGha = Math.cos(gha);
        var sinGha = Math.sin(gha);

        if (!defined(result)) {
            return new Matrix3(cosGha, sinGha, 0.0,
                              -sinGha, cosGha, 0.0,
                                  0.0,    0.0, 1.0);
        }
        result[0] = cosGha;
        result[1] = -sinGha;
        result[2] = 0.0;
        result[3] = sinGha;
        result[4] = cosGha;
        result[5] = 0.0;
        result[6] = 0.0;
        result[7] = 0.0;
        result[8] = 1.0;
        return result;
    };

    /**
     * The source of IAU 2006 XYS data, used for computing the transformation between the
     * Fixed and ICRF axes.
     * @type {Iau2006XysData}
     *
     * @see Transforms.computeIcrfToFixedMatrix
     * @see Transforms.computeFixedToIcrfMatrix
     *
     * @private
     */
    Transforms.iau2006XysData = new Iau2006XysData();

    /**
     * The source of Earth Orientation Parameters (EOP) data, used for computing the transformation
     * between the Fixed and ICRF axes.  By default, zero values are used for all EOP values,
     * yielding a reasonable but not completely accurate representation of the ICRF axes.
     * @type {EarthOrientationParameters}
     *
     * @see Transforms.computeIcrfToFixedMatrix
     * @see Transforms.computeFixedToIcrfMatrix
     *
     * @private
     */
    Transforms.earthOrientationParameters = EarthOrientationParameters.NONE;

    var ttMinusTai = 32.184;
    var j2000ttDays = 2451545.0;

    /**
     * Preloads the data necessary to transform between the ICRF and Fixed axes, in either
     * direction, over a given interval.  This function returns a promise that, when resolved,
     * indicates that the preload has completed.
     *
     * @param {TimeInterval} timeInterval The interval to preload.
     * @returns {Promise} A promise that, when resolved, indicates that the preload has completed
     *          and evaluation of the transformation between the fixed and ICRF axes will
     *          no longer return undefined for a time inside the interval.
     *
     *
     * @example
     * var interval = new Cesium.TimeInterval(...);
     * when(Cesium.Transforms.preloadIcrfFixed(interval), function() {
     *     // the data is now loaded
     * });
     *
     * @see Transforms.computeIcrfToFixedMatrix
     * @see Transforms.computeFixedToIcrfMatrix
     * @see when
     */
    Transforms.preloadIcrfFixed = function(timeInterval) {
        var startDayTT = timeInterval.start.dayNumber;
        var startSecondTT = timeInterval.start.secondsOfDay + ttMinusTai;
        var stopDayTT = timeInterval.stop.dayNumber;
        var stopSecondTT = timeInterval.stop.secondsOfDay + ttMinusTai;

        var xysPromise = Transforms.iau2006XysData.preload(startDayTT, startSecondTT, stopDayTT, stopSecondTT);
        var eopPromise = Transforms.earthOrientationParameters.getPromiseToLoad();

        return when.all([xysPromise, eopPromise]);
    };

    /**
     * Computes a rotation matrix to transform a point or vector from the International Celestial
     * Reference Frame (GCRF/ICRF) inertial frame axes to the Earth-Fixed frame axes (ITRF)
     * at a given time.  This function may return undefined if the data necessary to
     * do the transformation is not yet loaded.
     *
     * @param {JulianDate} date The time at which to compute the rotation matrix.
     * @param {Matrix3} [result] The object onto which to store the result.  If this parameter is
     *                  not specified, a new instance is created and returned.
     * @returns {Matrix3} The rotation matrix, or undefined if the data necessary to do the
     *                   transformation is not yet loaded.
     *
     *
     * @example
     * scene.postUpdate.addEventListener(function(scene, time) {
     *   // View in ICRF.
     *   var icrfToFixed = Cesium.Transforms.computeIcrfToFixedMatrix(time);
     *   if (Cesium.defined(icrfToFixed)) {
     *     var offset = Cesium.Cartesian3.clone(camera.position);
     *     var transform = Cesium.Matrix4.fromRotationTranslation(icrfToFixed);
     *     camera.lookAtTransform(transform, offset);
     *   }
     * });
     *
     * @see Transforms.preloadIcrfFixed
     */
    Transforms.computeIcrfToFixedMatrix = function(date, result) {
                if (!defined(date)) {
            throw new DeveloperError('date is required.');
        }
                if (!defined(result)) {
            result = new Matrix3();
        }

        var fixedToIcrfMtx = Transforms.computeFixedToIcrfMatrix(date, result);
        if (!defined(fixedToIcrfMtx)) {
            return undefined;
        }

        return Matrix3.transpose(fixedToIcrfMtx, result);
    };

    var xysScratch = new Iau2006XysSample(0.0, 0.0, 0.0);
    var eopScratch = new EarthOrientationParametersSample(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
    var rotation1Scratch = new Matrix3();
    var rotation2Scratch = new Matrix3();

    /**
     * Computes a rotation matrix to transform a point or vector from the Earth-Fixed frame axes (ITRF)
     * to the International Celestial Reference Frame (GCRF/ICRF) inertial frame axes
     * at a given time.  This function may return undefined if the data necessary to
     * do the transformation is not yet loaded.
     *
     * @param {JulianDate} date The time at which to compute the rotation matrix.
     * @param {Matrix3} [result] The object onto which to store the result.  If this parameter is
     *                  not specified, a new instance is created and returned.
     * @returns {Matrix3} The rotation matrix, or undefined if the data necessary to do the
     *                   transformation is not yet loaded.
     *
     *
     * @example
     * // Transform a point from the ICRF axes to the Fixed axes.
     * var now = Cesium.JulianDate.now();
     * var pointInFixed = Cesium.Cartesian3.fromDegrees(0.0, 0.0);
     * var fixedToIcrf = Cesium.Transforms.computeIcrfToFixedMatrix(now);
     * var pointInInertial = new Cesium.Cartesian3();
     * if (Cesium.defined(fixedToIcrf)) {
     *     pointInInertial = Cesium.Matrix3.multiplyByVector(fixedToIcrf, pointInFixed, pointInInertial);
     * }
     *
     * @see Transforms.preloadIcrfFixed
     */
    Transforms.computeFixedToIcrfMatrix = function(date, result) {
                if (!defined(date)) {
            throw new DeveloperError('date is required.');
        }
        
        if (!defined(result)) {
            result = new Matrix3();
        }

        // Compute pole wander
        var eop = Transforms.earthOrientationParameters.compute(date, eopScratch);
        if (!defined(eop)) {
            return undefined;
        }

        // There is no external conversion to Terrestrial Time (TT).
        // So use International Atomic Time (TAI) and convert using offsets.
        // Here we are assuming that dayTT and secondTT are positive
        var dayTT = date.dayNumber;
        // It's possible here that secondTT could roll over 86400
        // This does not seem to affect the precision (unit tests check for this)
        var secondTT = date.secondsOfDay + ttMinusTai;

        var xys = Transforms.iau2006XysData.computeXysRadians(dayTT, secondTT, xysScratch);
        if (!defined(xys)) {
            return undefined;
        }

        var x = xys.x + eop.xPoleOffset;
        var y = xys.y + eop.yPoleOffset;

        // Compute XYS rotation
        var a = 1.0 / (1.0 + Math.sqrt(1.0 - x * x - y * y));

        var rotation1 = rotation1Scratch;
        rotation1[0] = 1.0 - a * x * x;
        rotation1[3] = -a * x * y;
        rotation1[6] = x;
        rotation1[1] = -a * x * y;
        rotation1[4] = 1 - a * y * y;
        rotation1[7] = y;
        rotation1[2] = -x;
        rotation1[5] = -y;
        rotation1[8] = 1 - a * (x * x + y * y);

        var rotation2 = Matrix3.fromRotationZ(-xys.s, rotation2Scratch);
        var matrixQ = Matrix3.multiply(rotation1, rotation2, rotation1Scratch);

        // Similar to TT conversions above
        // It's possible here that secondTT could roll over 86400
        // This does not seem to affect the precision (unit tests check for this)
        var dateUt1day = date.dayNumber;
        var dateUt1sec = date.secondsOfDay - JulianDate.computeTaiMinusUtc(date) + eop.ut1MinusUtc;

        // Compute Earth rotation angle
        // The IERS standard for era is
        //    era = 0.7790572732640 + 1.00273781191135448 * Tu
        // where
        //    Tu = JulianDateInUt1 - 2451545.0
        // However, you get much more precision if you make the following simplification
        //    era = a + (1 + b) * (JulianDayNumber + FractionOfDay - 2451545)
        //    era = a + (JulianDayNumber - 2451545) + FractionOfDay + b (JulianDayNumber - 2451545 + FractionOfDay)
        //    era = a + FractionOfDay + b (JulianDayNumber - 2451545 + FractionOfDay)
        // since (JulianDayNumber - 2451545) represents an integer number of revolutions which will be discarded anyway.
        var daysSinceJ2000 = dateUt1day - 2451545;
        var fractionOfDay = dateUt1sec / TimeConstants.SECONDS_PER_DAY;
        var era = 0.7790572732640 + fractionOfDay + 0.00273781191135448 * (daysSinceJ2000 + fractionOfDay);
        era = (era % 1.0) * CesiumMath.TWO_PI;

        var earthRotation = Matrix3.fromRotationZ(era, rotation2Scratch);

        // pseudoFixed to ICRF
        var pfToIcrf = Matrix3.multiply(matrixQ, earthRotation, rotation1Scratch);

        // Compute pole wander matrix
        var cosxp = Math.cos(eop.xPoleWander);
        var cosyp = Math.cos(eop.yPoleWander);
        var sinxp = Math.sin(eop.xPoleWander);
        var sinyp = Math.sin(eop.yPoleWander);

        var ttt = (dayTT - j2000ttDays) + secondTT / TimeConstants.SECONDS_PER_DAY;
        ttt /= 36525.0;

        // approximate sp value in rad
        var sp = -47.0e-6 * ttt * CesiumMath.RADIANS_PER_DEGREE / 3600.0;
        var cossp = Math.cos(sp);
        var sinsp = Math.sin(sp);

        var fToPfMtx = rotation2Scratch;
        fToPfMtx[0] = cosxp * cossp;
        fToPfMtx[1] = cosxp * sinsp;
        fToPfMtx[2] = sinxp;
        fToPfMtx[3] = -cosyp * sinsp + sinyp * sinxp * cossp;
        fToPfMtx[4] = cosyp * cossp + sinyp * sinxp * sinsp;
        fToPfMtx[5] = -sinyp * cosxp;
        fToPfMtx[6] = -sinyp * sinsp - cosyp * sinxp * cossp;
        fToPfMtx[7] = sinyp * cossp - cosyp * sinxp * sinsp;
        fToPfMtx[8] = cosyp * cosxp;

        return Matrix3.multiply(pfToIcrf, fToPfMtx, result);
    };

    var pointToWindowCoordinatesTemp = new Cartesian4();

    /**
     * Transform a point from model coordinates to window coordinates.
     *
     * @param {Matrix4} modelViewProjectionMatrix The 4x4 model-view-projection matrix.
     * @param {Matrix4} viewportTransformation The 4x4 viewport transformation.
     * @param {Cartesian3} point The point to transform.
     * @param {Cartesian2} [result] The object onto which to store the result.
     * @returns {Cartesian2} The modified result parameter or a new Cartesian2 instance if none was provided.
     */
    Transforms.pointToWindowCoordinates = function (modelViewProjectionMatrix, viewportTransformation, point, result) {
        result = Transforms.pointToGLWindowCoordinates(modelViewProjectionMatrix, viewportTransformation, point, result);
        result.y = 2.0 * viewportTransformation[5] - result.y;
        return result;
    };

    /**
     * @private
     */
    Transforms.pointToGLWindowCoordinates = function(modelViewProjectionMatrix, viewportTransformation, point, result) {
                if (!defined(modelViewProjectionMatrix)) {
            throw new DeveloperError('modelViewProjectionMatrix is required.');
        }

        if (!defined(viewportTransformation)) {
            throw new DeveloperError('viewportTransformation is required.');
        }

        if (!defined(point)) {
            throw new DeveloperError('point is required.');
        }
        
        if (!defined(result)) {
            result = new Cartesian2();
        }

        var tmp = pointToWindowCoordinatesTemp;

        Matrix4.multiplyByVector(modelViewProjectionMatrix, Cartesian4.fromElements(point.x, point.y, point.z, 1, tmp), tmp);
        Cartesian4.multiplyByScalar(tmp, 1.0 / tmp.w, tmp);
        Matrix4.multiplyByVector(viewportTransformation, tmp, tmp);
        return Cartesian2.fromCartesian4(tmp, result);
    };

    var normalScratch = new Cartesian3();
    var rightScratch = new Cartesian3();
    var upScratch = new Cartesian3();

    /**
     * @private
     */
    Transforms.rotationMatrixFromPositionVelocity = function(position, velocity, ellipsoid, result) {
                if (!defined(position)) {
            throw new DeveloperError('position is required.');
        }

        if (!defined(velocity)) {
            throw new DeveloperError('velocity is required.');
        }
        
        var normal = defaultValue(ellipsoid, Ellipsoid.WGS84).geodeticSurfaceNormal(position, normalScratch);
        var right = Cartesian3.cross(velocity, normal, rightScratch);

        if (Cartesian3.equalsEpsilon(right, Cartesian3.ZERO, CesiumMath.EPSILON6)) {
            right = Cartesian3.clone(Cartesian3.UNIT_X, right);
        }

        var up = Cartesian3.cross(right, velocity, upScratch);
        Cartesian3.normalize(up, up);
        Cartesian3.cross(velocity, up, right);
        Cartesian3.negate(right, right);
        Cartesian3.normalize(right, right);

        if (!defined(result)) {
            result = new Matrix3();
        }

        result[0] = velocity.x;
        result[1] = velocity.y;
        result[2] = velocity.z;
        result[3] = right.x;
        result[4] = right.y;
        result[5] = right.z;
        result[6] = up.x;
        result[7] = up.y;
        result[8] = up.z;

        return result;
    };

    var swizzleMatrix = new Matrix4(
        0.0, 0.0, 1.0, 0.0,
        1.0, 0.0, 0.0, 0.0,
        0.0, 1.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 1.0
    );

    var scratchCartographic = new Cartographic();
    var scratchCartesian3Projection = new Cartesian3();
    var scratchCenter = new Cartesian3();
    var scratchRotation = new Matrix3();
    var scratchFromENU = new Matrix4();
    var scratchToENU = new Matrix4();

    /**
     * @private
     */
    Transforms.basisTo2D = function(projection, matrix, result) {
                if (!defined(projection)) {
            throw new DeveloperError('projection is required.');
        }
        if (!defined(matrix)) {
            throw new DeveloperError('matrix is required.');
        }
        if (!defined(result)) {
            throw new DeveloperError('result is required.');
        }
        
        var rtcCenter = Matrix4.getTranslation(matrix, scratchCenter);
        var ellipsoid = projection.ellipsoid;

        // Get the 2D Center
        var cartographic = ellipsoid.cartesianToCartographic(rtcCenter, scratchCartographic);
        var projectedPosition = projection.project(cartographic, scratchCartesian3Projection);
        Cartesian3.fromElements(projectedPosition.z, projectedPosition.x, projectedPosition.y, projectedPosition);

        // Assuming the instance are positioned in WGS84, invert the WGS84 transform to get the local transform and then convert to 2D
        var fromENU = Transforms.eastNorthUpToFixedFrame(rtcCenter, ellipsoid, scratchFromENU);
        var toENU = Matrix4.inverseTransformation(fromENU, scratchToENU);
        var rotation = Matrix4.getRotation(matrix, scratchRotation);
        var local = Matrix4.multiplyByMatrix3(toENU, rotation, result);
        Matrix4.multiply(swizzleMatrix, local, result); // Swap x, y, z for 2D
        Matrix4.setTranslation(result, projectedPosition, result); // Use the projected center

        return result;
    };

    /**
     * @private
     */
    Transforms.wgs84To2DModelMatrix = function(projection, center, result) {
                if (!defined(projection)) {
            throw new DeveloperError('projection is required.');
        }
        if (!defined(center)) {
            throw new DeveloperError('center is required.');
        }
        if (!defined(result)) {
            throw new DeveloperError('result is required.');
        }
        
        var ellipsoid = projection.ellipsoid;

        var fromENU = Transforms.eastNorthUpToFixedFrame(center, ellipsoid, scratchFromENU);
        var toENU = Matrix4.inverseTransformation(fromENU, scratchToENU);

        var cartographic = ellipsoid.cartesianToCartographic(center, scratchCartographic);
        var projectedPosition = projection.project(cartographic, scratchCartesian3Projection);
        Cartesian3.fromElements(projectedPosition.z, projectedPosition.x, projectedPosition.y, projectedPosition);

        var translation = Matrix4.fromTranslation(projectedPosition, scratchFromENU);
        Matrix4.multiply(swizzleMatrix, toENU, result);
        Matrix4.multiply(translation, result, result);

        return result;
    };

    return Transforms;
});

define('Core/Geometry',[
        './Cartesian2',
        './Cartesian3',
        './Cartographic',
        './Check',
        './defaultValue',
        './defined',
        './DeveloperError',
        './GeometryOffsetAttribute',
        './GeometryType',
        './Matrix2',
        './Matrix3',
        './Matrix4',
        './PrimitiveType',
        './Quaternion',
        './Rectangle',
        './Transforms'
    ], function(
        Cartesian2,
        Cartesian3,
        Cartographic,
        Check,
        defaultValue,
        defined,
        DeveloperError,
        GeometryOffsetAttribute,
        GeometryType,
        Matrix2,
        Matrix3,
        Matrix4,
        PrimitiveType,
        Quaternion,
        Rectangle,
        Transforms) {
    'use strict';

    /**
     * A geometry representation with attributes forming vertices and optional index data
     * defining primitives.  Geometries and an {@link Appearance}, which describes the shading,
     * can be assigned to a {@link Primitive} for visualization.  A <code>Primitive</code> can
     * be created from many heterogeneous - in many cases - geometries for performance.
     * <p>
     * Geometries can be transformed and optimized using functions in {@link GeometryPipeline}.
     * </p>
     *
     * @alias Geometry
     * @constructor
     *
     * @param {Object} options Object with the following properties:
     * @param {GeometryAttributes} options.attributes Attributes, which make up the geometry's vertices.
     * @param {PrimitiveType} [options.primitiveType=PrimitiveType.TRIANGLES] The type of primitives in the geometry.
     * @param {Uint16Array|Uint32Array} [options.indices] Optional index data that determines the primitives in the geometry.
     * @param {BoundingSphere} [options.boundingSphere] An optional bounding sphere that fully enclosed the geometry.
     *
     * @see PolygonGeometry
     * @see RectangleGeometry
     * @see EllipseGeometry
     * @see CircleGeometry
     * @see WallGeometry
     * @see SimplePolylineGeometry
     * @see BoxGeometry
     * @see EllipsoidGeometry
     *
     * @demo {@link https://cesiumjs.org/Cesium/Apps/Sandcastle/index.html?src=Geometry%20and%20Appearances.html|Geometry and Appearances Demo}
     *
     * @example
     * // Create geometry with a position attribute and indexed lines.
     * var positions = new Float64Array([
     *   0.0, 0.0, 0.0,
     *   7500000.0, 0.0, 0.0,
     *   0.0, 7500000.0, 0.0
     * ]);
     *
     * var geometry = new Cesium.Geometry({
     *   attributes : {
     *     position : new Cesium.GeometryAttribute({
     *       componentDatatype : Cesium.ComponentDatatype.DOUBLE,
     *       componentsPerAttribute : 3,
     *       values : positions
     *     })
     *   },
     *   indices : new Uint16Array([0, 1, 1, 2, 2, 0]),
     *   primitiveType : Cesium.PrimitiveType.LINES,
     *   boundingSphere : Cesium.BoundingSphere.fromVertices(positions)
     * });
     */
    function Geometry(options) {
        options = defaultValue(options, defaultValue.EMPTY_OBJECT);

                Check.typeOf.object('options.attributes', options.attributes);
        
        /**
         * Attributes, which make up the geometry's vertices.  Each property in this object corresponds to a
         * {@link GeometryAttribute} containing the attribute's data.
         * <p>
         * Attributes are always stored non-interleaved in a Geometry.
         * </p>
         * <p>
         * There are reserved attribute names with well-known semantics.  The following attributes
         * are created by a Geometry (depending on the provided {@link VertexFormat}.
         * <ul>
         *    <li><code>position</code> - 3D vertex position.  64-bit floating-point (for precision).  3 components per attribute.  See {@link VertexFormat#position}.</li>
         *    <li><code>normal</code> - Normal (normalized), commonly used for lighting.  32-bit floating-point.  3 components per attribute.  See {@link VertexFormat#normal}.</li>
         *    <li><code>st</code> - 2D texture coordinate.  32-bit floating-point.  2 components per attribute.  See {@link VertexFormat#st}.</li>
         *    <li><code>bitangent</code> - Bitangent (normalized), used for tangent-space effects like bump mapping.  32-bit floating-point.  3 components per attribute.  See {@link VertexFormat#bitangent}.</li>
         *    <li><code>tangent</code> - Tangent (normalized), used for tangent-space effects like bump mapping.  32-bit floating-point.  3 components per attribute.  See {@link VertexFormat#tangent}.</li>
         * </ul>
         * </p>
         * <p>
         * The following attribute names are generally not created by a Geometry, but are added
         * to a Geometry by a {@link Primitive} or {@link GeometryPipeline} functions to prepare
         * the geometry for rendering.
         * <ul>
         *    <li><code>position3DHigh</code> - High 32 bits for encoded 64-bit position computed with {@link GeometryPipeline.encodeAttribute}.  32-bit floating-point.  4 components per attribute.</li>
         *    <li><code>position3DLow</code> - Low 32 bits for encoded 64-bit position computed with {@link GeometryPipeline.encodeAttribute}.  32-bit floating-point.  4 components per attribute.</li>
         *    <li><code>position3DHigh</code> - High 32 bits for encoded 64-bit 2D (Columbus view) position computed with {@link GeometryPipeline.encodeAttribute}.  32-bit floating-point.  4 components per attribute.</li>
         *    <li><code>position2DLow</code> - Low 32 bits for encoded 64-bit 2D (Columbus view) position computed with {@link GeometryPipeline.encodeAttribute}.  32-bit floating-point.  4 components per attribute.</li>
         *    <li><code>color</code> - RGBA color (normalized) usually from {@link GeometryInstance#color}.  32-bit floating-point.  4 components per attribute.</li>
         *    <li><code>pickColor</code> - RGBA color used for picking.  32-bit floating-point.  4 components per attribute.</li>
         * </ul>
         * </p>
         *
         * @type GeometryAttributes
         *
         * @default undefined
         *
         *
         * @example
         * geometry.attributes.position = new Cesium.GeometryAttribute({
         *   componentDatatype : Cesium.ComponentDatatype.FLOAT,
         *   componentsPerAttribute : 3,
         *   values : new Float32Array(0)
         * });
         *
         * @see GeometryAttribute
         * @see VertexFormat
         */
        this.attributes = options.attributes;

        /**
         * Optional index data that - along with {@link Geometry#primitiveType} -
         * determines the primitives in the geometry.
         *
         * @type Array
         *
         * @default undefined
         */
        this.indices = options.indices;

        /**
         * The type of primitives in the geometry.  This is most often {@link PrimitiveType.TRIANGLES},
         * but can varying based on the specific geometry.
         *
         * @type PrimitiveType
         *
         * @default undefined
         */
        this.primitiveType = defaultValue(options.primitiveType, PrimitiveType.TRIANGLES);

        /**
         * An optional bounding sphere that fully encloses the geometry.  This is
         * commonly used for culling.
         *
         * @type BoundingSphere
         *
         * @default undefined
         */
        this.boundingSphere = options.boundingSphere;

        /**
         * @private
         */
        this.geometryType = defaultValue(options.geometryType, GeometryType.NONE);

        /**
         * @private
         */
        this.boundingSphereCV = options.boundingSphereCV;

        /**
         * @private
         * Used for computing the bounding sphere for geometry using the applyOffset vertex attribute
         */
        this.offsetAttribute = options.offsetAttribute;
    }

    /**
     * Computes the number of vertices in a geometry.  The runtime is linear with
     * respect to the number of attributes in a vertex, not the number of vertices.
     *
     * @param {Geometry} geometry The geometry.
     * @returns {Number} The number of vertices in the geometry.
     *
     * @example
     * var numVertices = Cesium.Geometry.computeNumberOfVertices(geometry);
     */
    Geometry.computeNumberOfVertices = function(geometry) {
                Check.typeOf.object('geometry', geometry);
        
        var numberOfVertices = -1;
        for ( var property in geometry.attributes) {
            if (geometry.attributes.hasOwnProperty(property) &&
                    defined(geometry.attributes[property]) &&
                    defined(geometry.attributes[property].values)) {

                var attribute = geometry.attributes[property];
                var num = attribute.values.length / attribute.componentsPerAttribute;
                                if ((numberOfVertices !== num) && (numberOfVertices !== -1)) {
                    throw new DeveloperError('All attribute lists must have the same number of attributes.');
                }
                                numberOfVertices = num;
            }
        }

        return numberOfVertices;
    };

    var rectangleCenterScratch = new Cartographic();
    var enuCenterScratch = new Cartesian3();
    var fixedFrameToEnuScratch = new Matrix4();
    var boundingRectanglePointsCartographicScratch = [new Cartographic(), new Cartographic(), new Cartographic()];
    var boundingRectanglePointsEnuScratch = [new Cartesian2(), new Cartesian2(), new Cartesian2()];
    var points2DScratch = [new Cartesian2(), new Cartesian2(), new Cartesian2()];
    var pointEnuScratch = new Cartesian3();
    var enuRotationScratch = new Quaternion();
    var enuRotationMatrixScratch = new Matrix4();
    var rotation2DScratch = new Matrix2();

    /**
     * For remapping texture coordinates when rendering GroundPrimitives with materials.
     * GroundPrimitive texture coordinates are computed to align with the cartographic coordinate system on the globe.
     * However, EllipseGeometry, RectangleGeometry, and PolygonGeometry all bake rotations to per-vertex texture coordinates
     * using different strategies.
     *
     * This method is used by EllipseGeometry and PolygonGeometry to approximate the same visual effect.
     * We encapsulate rotation and scale by computing a "transformed" texture coordinate system and computing
     * a set of reference points from which "cartographic" texture coordinates can be remapped to the "transformed"
     * system using distances to lines in 2D.
     *
     * This approximation becomes less accurate as the covered area increases, especially for GroundPrimitives near the poles,
     * but is generally reasonable for polygons and ellipses around the size of USA states.
     *
     * RectangleGeometry has its own version of this method that computes remapping coordinates using cartographic space
     * as an intermediary instead of local ENU, which is more accurate for large-area rectangles.
     *
     * @param {Cartesian3[]} positions Array of positions outlining the geometry
     * @param {Number} stRotation Texture coordinate rotation.
     * @param {Ellipsoid} ellipsoid Ellipsoid for projecting and generating local vectors.
     * @param {Rectangle} boundingRectangle Bounding rectangle around the positions.
     * @returns {Number[]} An array of 6 numbers specifying [minimum point, u extent, v extent] as points in the "cartographic" system.
     * @private
     */
    Geometry._textureCoordinateRotationPoints = function(positions, stRotation, ellipsoid, boundingRectangle) {
        var i;

        // Create a local east-north-up coordinate system centered on the polygon's bounding rectangle.
        // Project the southwest, northwest, and southeast corners of the bounding rectangle into the plane of ENU as 2D points.
        // These are the equivalents of (0,0), (0,1), and (1,0) in the texture coordiante system computed in ShadowVolumeAppearanceFS,
        // aka "ENU texture space."
        var rectangleCenter = Rectangle.center(boundingRectangle, rectangleCenterScratch);
        var enuCenter = Cartographic.toCartesian(rectangleCenter, ellipsoid, enuCenterScratch);
        var enuToFixedFrame = Transforms.eastNorthUpToFixedFrame(enuCenter, ellipsoid, fixedFrameToEnuScratch);
        var fixedFrameToEnu = Matrix4.inverse(enuToFixedFrame, fixedFrameToEnuScratch);

        var boundingPointsEnu = boundingRectanglePointsEnuScratch;
        var boundingPointsCarto = boundingRectanglePointsCartographicScratch;

        boundingPointsCarto[0].longitude = boundingRectangle.west;
        boundingPointsCarto[0].latitude = boundingRectangle.south;

        boundingPointsCarto[1].longitude = boundingRectangle.west;
        boundingPointsCarto[1].latitude = boundingRectangle.north;

        boundingPointsCarto[2].longitude = boundingRectangle.east;
        boundingPointsCarto[2].latitude = boundingRectangle.south;

        var posEnu = pointEnuScratch;

        for (i = 0; i < 3; i++) {
            Cartographic.toCartesian(boundingPointsCarto[i], ellipsoid, posEnu);
            posEnu = Matrix4.multiplyByPointAsVector(fixedFrameToEnu, posEnu, posEnu);
            boundingPointsEnu[i].x = posEnu.x;
            boundingPointsEnu[i].y = posEnu.y;
        }

        // Rotate each point in the polygon around the up vector in the ENU by -stRotation and project into ENU as 2D.
        // Compute the bounding box of these rotated points in the 2D ENU plane.
        // Rotate the corners back by stRotation, then compute their equivalents in the ENU texture space using the corners computed earlier.
        var rotation = Quaternion.fromAxisAngle(Cartesian3.UNIT_Z, -stRotation, enuRotationScratch);
        var textureMatrix = Matrix3.fromQuaternion(rotation, enuRotationMatrixScratch);

        var positionsLength = positions.length;
        var enuMinX = Number.POSITIVE_INFINITY;
        var enuMinY = Number.POSITIVE_INFINITY;
        var enuMaxX = Number.NEGATIVE_INFINITY;
        var enuMaxY = Number.NEGATIVE_INFINITY;
        for (i = 0; i < positionsLength; i++) {
            posEnu = Matrix4.multiplyByPointAsVector(fixedFrameToEnu, positions[i], posEnu);
            posEnu = Matrix3.multiplyByVector(textureMatrix, posEnu, posEnu);

            enuMinX = Math.min(enuMinX, posEnu.x);
            enuMinY = Math.min(enuMinY, posEnu.y);
            enuMaxX = Math.max(enuMaxX, posEnu.x);
            enuMaxY = Math.max(enuMaxY, posEnu.y);
        }

        var toDesiredInComputed = Matrix2.fromRotation(stRotation, rotation2DScratch);

        var points2D = points2DScratch;
        points2D[0].x = enuMinX;
        points2D[0].y = enuMinY;

        points2D[1].x = enuMinX;
        points2D[1].y = enuMaxY;

        points2D[2].x = enuMaxX;
        points2D[2].y = enuMinY;

        var boundingEnuMin = boundingPointsEnu[0];
        var boundingPointsWidth = boundingPointsEnu[2].x - boundingEnuMin.x;
        var boundingPointsHeight = boundingPointsEnu[1].y - boundingEnuMin.y;

        for (i = 0; i < 3; i++) {
            var point2D = points2D[i];
            // rotate back
            Matrix2.multiplyByVector(toDesiredInComputed, point2D, point2D);

            // Convert point into east-north texture coordinate space
            point2D.x = (point2D.x - boundingEnuMin.x) / boundingPointsWidth;
            point2D.y = (point2D.y - boundingEnuMin.y) / boundingPointsHeight;
        }

        var minXYCorner = points2D[0];
        var maxYCorner = points2D[1];
        var maxXCorner = points2D[2];
        var result = new Array(6);
        Cartesian2.pack(minXYCorner, result);
        Cartesian2.pack(maxYCorner, result, 2);
        Cartesian2.pack(maxXCorner, result, 4);

        return result;
    };

    return Geometry;
});

define('Core/GeometryAttribute',[
        './defaultValue',
        './defined',
        './DeveloperError'
    ], function(
        defaultValue,
        defined,
        DeveloperError) {
    'use strict';

    /**
     * Values and type information for geometry attributes.  A {@link Geometry}
     * generally contains one or more attributes.  All attributes together form
     * the geometry's vertices.
     *
     * @alias GeometryAttribute
     * @constructor
     *
     * @param {Object} [options] Object with the following properties:
     * @param {ComponentDatatype} [options.componentDatatype] The datatype of each component in the attribute, e.g., individual elements in values.
     * @param {Number} [options.componentsPerAttribute] A number between 1 and 4 that defines the number of components in an attributes.
     * @param {Boolean} [options.normalize=false] When <code>true</code> and <code>componentDatatype</code> is an integer format, indicate that the components should be mapped to the range [0, 1] (unsigned) or [-1, 1] (signed) when they are accessed as floating-point for rendering.
     * @param {TypedArray} [options.values] The values for the attributes stored in a typed array.
     *
     * @exception {DeveloperError} options.componentsPerAttribute must be between 1 and 4.
     *
     *
     * @example
     * var geometry = new Cesium.Geometry({
     *   attributes : {
     *     position : new Cesium.GeometryAttribute({
     *       componentDatatype : Cesium.ComponentDatatype.FLOAT,
     *       componentsPerAttribute : 3,
     *       values : new Float32Array([
     *         0.0, 0.0, 0.0,
     *         7500000.0, 0.0, 0.0,
     *         0.0, 7500000.0, 0.0
     *       ])
     *     })
     *   },
     *   primitiveType : Cesium.PrimitiveType.LINE_LOOP
     * });
     *
     * @see Geometry
     */
    function GeometryAttribute(options) {
        options = defaultValue(options, defaultValue.EMPTY_OBJECT);

                if (!defined(options.componentDatatype)) {
            throw new DeveloperError('options.componentDatatype is required.');
        }
        if (!defined(options.componentsPerAttribute)) {
            throw new DeveloperError('options.componentsPerAttribute is required.');
        }
        if (options.componentsPerAttribute < 1 || options.componentsPerAttribute > 4) {
            throw new DeveloperError('options.componentsPerAttribute must be between 1 and 4.');
        }
        if (!defined(options.values)) {
            throw new DeveloperError('options.values is required.');
        }
        
        /**
         * The datatype of each component in the attribute, e.g., individual elements in
         * {@link GeometryAttribute#values}.
         *
         * @type ComponentDatatype
         *
         * @default undefined
         */
        this.componentDatatype = options.componentDatatype;

        /**
         * A number between 1 and 4 that defines the number of components in an attributes.
         * For example, a position attribute with x, y, and z components would have 3 as
         * shown in the code example.
         *
         * @type Number
         *
         * @default undefined
         *
         * @example
         * attribute.componentDatatype = Cesium.ComponentDatatype.FLOAT;
         * attribute.componentsPerAttribute = 3;
         * attribute.values = new Float32Array([
         *   0.0, 0.0, 0.0,
         *   7500000.0, 0.0, 0.0,
         *   0.0, 7500000.0, 0.0
         * ]);
         */
        this.componentsPerAttribute = options.componentsPerAttribute;

        /**
         * When <code>true</code> and <code>componentDatatype</code> is an integer format,
         * indicate that the components should be mapped to the range [0, 1] (unsigned)
         * or [-1, 1] (signed) when they are accessed as floating-point for rendering.
         * <p>
         * This is commonly used when storing colors using {@link ComponentDatatype.UNSIGNED_BYTE}.
         * </p>
         *
         * @type Boolean
         *
         * @default false
         *
         * @example
         * attribute.componentDatatype = Cesium.ComponentDatatype.UNSIGNED_BYTE;
         * attribute.componentsPerAttribute = 4;
         * attribute.normalize = true;
         * attribute.values = new Uint8Array([
         *   Cesium.Color.floatToByte(color.red),
         *   Cesium.Color.floatToByte(color.green),
         *   Cesium.Color.floatToByte(color.blue),
         *   Cesium.Color.floatToByte(color.alpha)
         * ]);
         */
        this.normalize = defaultValue(options.normalize, false);

        /**
         * The values for the attributes stored in a typed array.  In the code example,
         * every three elements in <code>values</code> defines one attributes since
         * <code>componentsPerAttribute</code> is 3.
         *
         * @type TypedArray
         *
         * @default undefined
         *
         * @example
         * attribute.componentDatatype = Cesium.ComponentDatatype.FLOAT;
         * attribute.componentsPerAttribute = 3;
         * attribute.values = new Float32Array([
         *   0.0, 0.0, 0.0,
         *   7500000.0, 0.0, 0.0,
         *   0.0, 7500000.0, 0.0
         * ]);
         */
        this.values = options.values;
    }

    return GeometryAttribute;
});

define('Core/GeometryAttributes',[
        './defaultValue'
    ], function(
        defaultValue) {
    'use strict';

    /**
     * Attributes, which make up a geometry's vertices.  Each property in this object corresponds to a
     * {@link GeometryAttribute} containing the attribute's data.
     * <p>
     * Attributes are always stored non-interleaved in a Geometry.
     * </p>
     *
     * @alias GeometryAttributes
     * @constructor
     */
    function GeometryAttributes(options) {
        options = defaultValue(options, defaultValue.EMPTY_OBJECT);

        /**
         * The 3D position attribute.
         * <p>
         * 64-bit floating-point (for precision).  3 components per attribute.
         * </p>
         *
         * @type GeometryAttribute
         *
         * @default undefined
         */
        this.position = options.position;

        /**
         * The normal attribute (normalized), which is commonly used for lighting.
         * <p>
         * 32-bit floating-point.  3 components per attribute.
         * </p>
         *
         * @type GeometryAttribute
         *
         * @default undefined
         */
        this.normal = options.normal;

        /**
         * The 2D texture coordinate attribute.
         * <p>
         * 32-bit floating-point.  2 components per attribute
         * </p>
         *
         * @type GeometryAttribute
         *
         * @default undefined
         */
        this.st = options.st;

        /**
         * The bitangent attribute (normalized), which is used for tangent-space effects like bump mapping.
         * <p>
         * 32-bit floating-point.  3 components per attribute.
         * </p>
         *
         * @type GeometryAttribute
         *
         * @default undefined
         */
        this.bitangent = options.bitangent;

        /**
         * The tangent attribute (normalized), which is used for tangent-space effects like bump mapping.
         * <p>
         * 32-bit floating-point.  3 components per attribute.
         * </p>
         *
         * @type GeometryAttribute
         *
         * @default undefined
         */
        this.tangent = options.tangent;

        /**
         * The color attribute.
         * <p>
         * 8-bit unsigned integer. 4 components per attribute.
         * </p>
         *
         * @type GeometryAttribute
         *
         * @default undefined
         */
        this.color = options.color;
    }

    return GeometryAttributes;
});

define('Core/IndexDatatype',[
        './defined',
        './DeveloperError',
        './freezeObject',
        './Math',
        './WebGLConstants'
    ], function(
        defined,
        DeveloperError,
        freezeObject,
        CesiumMath,
        WebGLConstants) {
    'use strict';

    /**
     * Constants for WebGL index datatypes.  These corresponds to the
     * <code>type</code> parameter of {@link http://www.khronos.org/opengles/sdk/docs/man/xhtml/glDrawElements.xml|drawElements}.
     *
     * @exports IndexDatatype
     */
    var IndexDatatype = {
        /**
         * 8-bit unsigned byte corresponding to <code>UNSIGNED_BYTE</code> and the type
         * of an element in <code>Uint8Array</code>.
         *
         * @type {Number}
         * @constant
         */
        UNSIGNED_BYTE : WebGLConstants.UNSIGNED_BYTE,

        /**
         * 16-bit unsigned short corresponding to <code>UNSIGNED_SHORT</code> and the type
         * of an element in <code>Uint16Array</code>.
         *
         * @type {Number}
         * @constant
         */
        UNSIGNED_SHORT : WebGLConstants.UNSIGNED_SHORT,

        /**
         * 32-bit unsigned int corresponding to <code>UNSIGNED_INT</code> and the type
         * of an element in <code>Uint32Array</code>.
         *
         * @type {Number}
         * @constant
         */
        UNSIGNED_INT : WebGLConstants.UNSIGNED_INT
    };

    /**
     * Returns the size, in bytes, of the corresponding datatype.
     *
     * @param {IndexDatatype} indexDatatype The index datatype to get the size of.
     * @returns {Number} The size in bytes.
     *
     * @example
     * // Returns 2
     * var size = Cesium.IndexDatatype.getSizeInBytes(Cesium.IndexDatatype.UNSIGNED_SHORT);
     */
    IndexDatatype.getSizeInBytes = function(indexDatatype) {
        switch(indexDatatype) {
            case IndexDatatype.UNSIGNED_BYTE:
                return Uint8Array.BYTES_PER_ELEMENT;
            case IndexDatatype.UNSIGNED_SHORT:
                return Uint16Array.BYTES_PER_ELEMENT;
            case IndexDatatype.UNSIGNED_INT:
                return Uint32Array.BYTES_PER_ELEMENT;
        }

                throw new DeveloperError('indexDatatype is required and must be a valid IndexDatatype constant.');
            };

    /**
     * Gets the datatype with a given size in bytes.
     *
     * @param {Number} sizeInBytes The size of a single index in bytes.
     * @returns {IndexDatatype} The index datatype with the given size.
     */
    IndexDatatype.fromSizeInBytes = function(sizeInBytes) {
        switch (sizeInBytes) {
            case 2:
                return IndexDatatype.UNSIGNED_SHORT;
            case 4:
                return IndexDatatype.UNSIGNED_INT;
            case 1:
                return IndexDatatype.UNSIGNED_BYTE;
                        default:
                throw new DeveloperError('Size in bytes cannot be mapped to an IndexDatatype');
                    }
    };

    /**
     * Validates that the provided index datatype is a valid {@link IndexDatatype}.
     *
     * @param {IndexDatatype} indexDatatype The index datatype to validate.
     * @returns {Boolean} <code>true</code> if the provided index datatype is a valid value; otherwise, <code>false</code>.
     *
     * @example
     * if (!Cesium.IndexDatatype.validate(indexDatatype)) {
     *   throw new Cesium.DeveloperError('indexDatatype must be a valid value.');
     * }
     */
    IndexDatatype.validate = function(indexDatatype) {
        return defined(indexDatatype) &&
               (indexDatatype === IndexDatatype.UNSIGNED_BYTE ||
                indexDatatype === IndexDatatype.UNSIGNED_SHORT ||
                indexDatatype === IndexDatatype.UNSIGNED_INT);
    };

    /**
     * Creates a typed array that will store indices, using either <code><Uint16Array</code>
     * or <code>Uint32Array</code> depending on the number of vertices.
     *
     * @param {Number} numberOfVertices Number of vertices that the indices will reference.
     * @param {Number|Array} indicesLengthOrArray Passed through to the typed array constructor.
     * @returns {Uint16Array|Uint32Array} A <code>Uint16Array</code> or <code>Uint32Array</code> constructed with <code>indicesLengthOrArray</code>.
     *
     * @example
     * this.indices = Cesium.IndexDatatype.createTypedArray(positions.length / 3, numberOfIndices);
     */
    IndexDatatype.createTypedArray = function(numberOfVertices, indicesLengthOrArray) {
                if (!defined(numberOfVertices)) {
            throw new DeveloperError('numberOfVertices is required.');
        }
        
        if (numberOfVertices >= CesiumMath.SIXTY_FOUR_KILOBYTES) {
            return new Uint32Array(indicesLengthOrArray);
        }

        return new Uint16Array(indicesLengthOrArray);
    };

    /**
     * Creates a typed array from a source array buffer.  The resulting typed array will store indices, using either <code><Uint16Array</code>
     * or <code>Uint32Array</code> depending on the number of vertices.
     *
     * @param {Number} numberOfVertices Number of vertices that the indices will reference.
     * @param {ArrayBuffer} sourceArray Passed through to the typed array constructor.
     * @param {Number} byteOffset Passed through to the typed array constructor.
     * @param {Number} length Passed through to the typed array constructor.
     * @returns {Uint16Array|Uint32Array} A <code>Uint16Array</code> or <code>Uint32Array</code> constructed with <code>sourceArray</code>, <code>byteOffset</code>, and <code>length</code>.
     *
     */
    IndexDatatype.createTypedArrayFromArrayBuffer = function(numberOfVertices, sourceArray, byteOffset, length) {
                if (!defined(numberOfVertices)) {
            throw new DeveloperError('numberOfVertices is required.');
        }
        if (!defined(sourceArray)) {
            throw new DeveloperError('sourceArray is required.');
        }
        if (!defined(byteOffset)) {
            throw new DeveloperError('byteOffset is required.');
        }
        
        if (numberOfVertices >= CesiumMath.SIXTY_FOUR_KILOBYTES) {
            return new Uint32Array(sourceArray, byteOffset, length);
        }

        return new Uint16Array(sourceArray, byteOffset, length);
    };

    return freezeObject(IndexDatatype);
});

define('Core/EllipseOutlineGeometry',[
        './arrayFill',
        './BoundingSphere',
        './Cartesian3',
        './ComponentDatatype',
        './defaultValue',
        './defined',
        './DeveloperError',
        './EllipseGeometryLibrary',
        './Ellipsoid',
        './Geometry',
        './GeometryAttribute',
        './GeometryAttributes',
        './GeometryOffsetAttribute',
        './IndexDatatype',
        './Math',
        './PrimitiveType'
    ], function(
        arrayFill,
        BoundingSphere,
        Cartesian3,
        ComponentDatatype,
        defaultValue,
        defined,
        DeveloperError,
        EllipseGeometryLibrary,
        Ellipsoid,
        Geometry,
        GeometryAttribute,
        GeometryAttributes,
        GeometryOffsetAttribute,
        IndexDatatype,
        CesiumMath,
        PrimitiveType) {
    'use strict';

    var scratchCartesian1 = new Cartesian3();
    var boundingSphereCenter = new Cartesian3();

    function computeEllipse(options) {
        var center = options.center;
        boundingSphereCenter = Cartesian3.multiplyByScalar(options.ellipsoid.geodeticSurfaceNormal(center, boundingSphereCenter), options.height, boundingSphereCenter);
        boundingSphereCenter = Cartesian3.add(center, boundingSphereCenter, boundingSphereCenter);
        var boundingSphere = new BoundingSphere(boundingSphereCenter, options.semiMajorAxis);
        var positions = EllipseGeometryLibrary.computeEllipsePositions(options, false, true).outerPositions;

        var attributes = new GeometryAttributes({
            position: new GeometryAttribute({
                componentDatatype : ComponentDatatype.DOUBLE,
                componentsPerAttribute : 3,
                values : EllipseGeometryLibrary.raisePositionsToHeight(positions, options, false)
            })
        });

        var length = positions.length / 3;
        var indices = IndexDatatype.createTypedArray(length, length * 2);
        var index = 0;
        for ( var i = 0; i < length; ++i) {
            indices[index++] = i;
            indices[index++] = (i + 1) % length;
        }

        return {
            boundingSphere : boundingSphere,
            attributes : attributes,
            indices : indices
        };
    }

    var topBoundingSphere = new BoundingSphere();
    var bottomBoundingSphere = new BoundingSphere();
    function computeExtrudedEllipse(options) {
        var center = options.center;
        var ellipsoid = options.ellipsoid;
        var semiMajorAxis = options.semiMajorAxis;
        var scaledNormal = Cartesian3.multiplyByScalar(ellipsoid.geodeticSurfaceNormal(center, scratchCartesian1), options.height, scratchCartesian1);
        topBoundingSphere.center = Cartesian3.add(center, scaledNormal, topBoundingSphere.center);
        topBoundingSphere.radius = semiMajorAxis;

        scaledNormal = Cartesian3.multiplyByScalar(ellipsoid.geodeticSurfaceNormal(center, scaledNormal), options.extrudedHeight, scaledNormal);
        bottomBoundingSphere.center = Cartesian3.add(center, scaledNormal, bottomBoundingSphere.center);
        bottomBoundingSphere.radius = semiMajorAxis;

        var positions = EllipseGeometryLibrary.computeEllipsePositions(options, false, true).outerPositions;
        var attributes = new GeometryAttributes({
            position: new GeometryAttribute({
                componentDatatype : ComponentDatatype.DOUBLE,
                componentsPerAttribute : 3,
                values : EllipseGeometryLibrary.raisePositionsToHeight(positions, options, true)
            })
        });

        positions = attributes.position.values;
        var boundingSphere = BoundingSphere.union(topBoundingSphere, bottomBoundingSphere);
        var length = positions.length/3;

        if (defined(options.offsetAttribute)) {
            var applyOffset = new Uint8Array(length);
            if (options.offsetAttribute === GeometryOffsetAttribute.TOP) {
                applyOffset = arrayFill(applyOffset, 1, 0, length / 2);
            } else {
                var offsetValue = options.offsetAttribute === GeometryOffsetAttribute.NONE ? 0 : 1;
                applyOffset = arrayFill(applyOffset, offsetValue);
            }

            attributes.applyOffset = new GeometryAttribute({
                componentDatatype : ComponentDatatype.UNSIGNED_BYTE,
                componentsPerAttribute : 1,
                values: applyOffset
            });
        }

        var numberOfVerticalLines = defaultValue(options.numberOfVerticalLines, 16);
        numberOfVerticalLines = CesiumMath.clamp(numberOfVerticalLines, 0, length/2);

        var indices = IndexDatatype.createTypedArray(length, length * 2 + numberOfVerticalLines * 2);

        length /= 2;
        var index = 0;
        var i;
        for (i = 0; i < length; ++i) {
            indices[index++] = i;
            indices[index++] = (i + 1) % length;
            indices[index++] = i + length;
            indices[index++] = ((i + 1) % length) + length;
        }

        var numSide;
        if (numberOfVerticalLines > 0) {
            var numSideLines = Math.min(numberOfVerticalLines, length);
            numSide = Math.round(length / numSideLines);

            var maxI = Math.min(numSide * numberOfVerticalLines, length);
            for (i = 0; i < maxI; i += numSide) {
                indices[index++] = i;
                indices[index++] = i + length;
            }
        }

        return {
            boundingSphere : boundingSphere,
            attributes : attributes,
            indices : indices
        };
    }

    /**
     * A description of the outline of an ellipse on an ellipsoid.
     *
     * @alias EllipseOutlineGeometry
     * @constructor
     *
     * @param {Object} options Object with the following properties:
     * @param {Cartesian3} options.center The ellipse's center point in the fixed frame.
     * @param {Number} options.semiMajorAxis The length of the ellipse's semi-major axis in meters.
     * @param {Number} options.semiMinorAxis The length of the ellipse's semi-minor axis in meters.
     * @param {Ellipsoid} [options.ellipsoid=Ellipsoid.WGS84] The ellipsoid the ellipse will be on.
     * @param {Number} [options.height=0.0] The distance in meters between the ellipse and the ellipsoid surface.
     * @param {Number} [options.extrudedHeight] The distance in meters between the ellipse's extruded face and the ellipsoid surface.
     * @param {Number} [options.rotation=0.0] The angle from north (counter-clockwise) in radians.
     * @param {Number} [options.granularity=0.02] The angular distance between points on the ellipse in radians.
     * @param {Number} [options.numberOfVerticalLines=16] Number of lines to draw between the top and bottom surface of an extruded ellipse.
     *
     * @exception {DeveloperError} semiMajorAxis and semiMinorAxis must be greater than zero.
     * @exception {DeveloperError} semiMajorAxis must be greater than or equal to the semiMinorAxis.
     * @exception {DeveloperError} granularity must be greater than zero.
     *
     * @see EllipseOutlineGeometry.createGeometry
     *
     * @example
     * var ellipse = new Cesium.EllipseOutlineGeometry({
     *   center : Cesium.Cartesian3.fromDegrees(-75.59777, 40.03883),
     *   semiMajorAxis : 500000.0,
     *   semiMinorAxis : 300000.0,
     *   rotation : Cesium.Math.toRadians(60.0)
     * });
     * var geometry = Cesium.EllipseOutlineGeometry.createGeometry(ellipse);
     */
    function EllipseOutlineGeometry(options) {
        options = defaultValue(options, defaultValue.EMPTY_OBJECT);

        var center = options.center;
        var ellipsoid = defaultValue(options.ellipsoid, Ellipsoid.WGS84);
        var semiMajorAxis = options.semiMajorAxis;
        var semiMinorAxis = options.semiMinorAxis;
        var granularity = defaultValue(options.granularity, CesiumMath.RADIANS_PER_DEGREE);

                if (!defined(center)) {
            throw new DeveloperError('center is required.');
        }
        if (!defined(semiMajorAxis)) {
            throw new DeveloperError('semiMajorAxis is required.');
        }
        if (!defined(semiMinorAxis)) {
            throw new DeveloperError('semiMinorAxis is required.');
        }
        if (semiMajorAxis < semiMinorAxis) {
            throw new DeveloperError('semiMajorAxis must be greater than or equal to the semiMinorAxis.');
        }
        if (granularity <= 0.0) {
            throw new DeveloperError('granularity must be greater than zero.');
        }
        
        var height = defaultValue(options.height, 0.0);
        var extrudedHeight = defaultValue(options.extrudedHeight, height);

        this._center = Cartesian3.clone(center);
        this._semiMajorAxis = semiMajorAxis;
        this._semiMinorAxis = semiMinorAxis;
        this._ellipsoid = Ellipsoid.clone(ellipsoid);
        this._rotation = defaultValue(options.rotation, 0.0);
        this._height = Math.max(extrudedHeight, height);
        this._granularity = granularity;
        this._extrudedHeight = Math.min(extrudedHeight, height);
        this._numberOfVerticalLines = Math.max(defaultValue(options.numberOfVerticalLines, 16), 0);
        this._offsetAttribute = options.offsetAttribute;
        this._workerName = 'createEllipseOutlineGeometry';
    }

    /**
     * The number of elements used to pack the object into an array.
     * @type {Number}
     */
    EllipseOutlineGeometry.packedLength = Cartesian3.packedLength + Ellipsoid.packedLength + 8;

    /**
     * Stores the provided instance into the provided array.
     *
     * @param {EllipseOutlineGeometry} value The value to pack.
     * @param {Number[]} array The array to pack into.
     * @param {Number} [startingIndex=0] The index into the array at which to start packing the elements.
     *
     * @returns {Number[]} The array that was packed into
     */
    EllipseOutlineGeometry.pack = function(value, array, startingIndex) {
                if (!defined(value)) {
            throw new DeveloperError('value is required');
        }
        if (!defined(array)) {
            throw new DeveloperError('array is required');
        }
        
        startingIndex = defaultValue(startingIndex, 0);

        Cartesian3.pack(value._center, array, startingIndex);
        startingIndex += Cartesian3.packedLength;

        Ellipsoid.pack(value._ellipsoid, array, startingIndex);
        startingIndex += Ellipsoid.packedLength;

        array[startingIndex++] = value._semiMajorAxis;
        array[startingIndex++] = value._semiMinorAxis;
        array[startingIndex++] = value._rotation;
        array[startingIndex++] = value._height;
        array[startingIndex++] = value._granularity;
        array[startingIndex++] = value._extrudedHeight;
        array[startingIndex++]   = value._numberOfVerticalLines;
        array[startingIndex] = defaultValue(value._offsetAttribute, -1);

        return array;
    };

    var scratchCenter = new Cartesian3();
    var scratchEllipsoid = new Ellipsoid();
    var scratchOptions = {
        center : scratchCenter,
        ellipsoid : scratchEllipsoid,
        semiMajorAxis : undefined,
        semiMinorAxis : undefined,
        rotation : undefined,
        height : undefined,
        granularity : undefined,
        extrudedHeight : undefined,
        numberOfVerticalLines : undefined,
        offsetAttribute: undefined
    };

    /**
     * Retrieves an instance from a packed array.
     *
     * @param {Number[]} array The packed array.
     * @param {Number} [startingIndex=0] The starting index of the element to be unpacked.
     * @param {EllipseOutlineGeometry} [result] The object into which to store the result.
     * @returns {EllipseOutlineGeometry} The modified result parameter or a new EllipseOutlineGeometry instance if one was not provided.
     */
    EllipseOutlineGeometry.unpack = function(array, startingIndex, result) {
                if (!defined(array)) {
            throw new DeveloperError('array is required');
        }
        
        startingIndex = defaultValue(startingIndex, 0);

        var center = Cartesian3.unpack(array, startingIndex, scratchCenter);
        startingIndex += Cartesian3.packedLength;

        var ellipsoid = Ellipsoid.unpack(array, startingIndex, scratchEllipsoid);
        startingIndex += Ellipsoid.packedLength;

        var semiMajorAxis = array[startingIndex++];
        var semiMinorAxis = array[startingIndex++];
        var rotation = array[startingIndex++];
        var height = array[startingIndex++];
        var granularity = array[startingIndex++];
        var extrudedHeight = array[startingIndex++];
        var numberOfVerticalLines = array[startingIndex++];
        var offsetAttribute = array[startingIndex];

        if (!defined(result)) {
            scratchOptions.height = height;
            scratchOptions.extrudedHeight = extrudedHeight;
            scratchOptions.granularity = granularity;
            scratchOptions.rotation = rotation;
            scratchOptions.semiMajorAxis = semiMajorAxis;
            scratchOptions.semiMinorAxis = semiMinorAxis;
            scratchOptions.numberOfVerticalLines = numberOfVerticalLines;
            scratchOptions.offsetAttribute = offsetAttribute === -1 ? undefined : offsetAttribute;

            return new EllipseOutlineGeometry(scratchOptions);
        }

        result._center = Cartesian3.clone(center, result._center);
        result._ellipsoid = Ellipsoid.clone(ellipsoid, result._ellipsoid);
        result._semiMajorAxis = semiMajorAxis;
        result._semiMinorAxis = semiMinorAxis;
        result._rotation = rotation;
        result._height = height;
        result._granularity = granularity;
        result._extrudedHeight = extrudedHeight;
        result._numberOfVerticalLines = numberOfVerticalLines;
        result._offsetAttribute = offsetAttribute === -1 ? undefined : offsetAttribute;

        return result;
    };

    /**
     * Computes the geometric representation of an outline of an ellipse on an ellipsoid, including its vertices, indices, and a bounding sphere.
     *
     * @param {EllipseOutlineGeometry} ellipseGeometry A description of the ellipse.
     * @returns {Geometry|undefined} The computed vertices and indices.
     */
    EllipseOutlineGeometry.createGeometry = function(ellipseGeometry) {
        if ((ellipseGeometry._semiMajorAxis <= 0.0) || (ellipseGeometry._semiMinorAxis <= 0.0)) {
            return;
        }

        var height = ellipseGeometry._height;
        var extrudedHeight = ellipseGeometry._extrudedHeight;
        var extrude = !CesiumMath.equalsEpsilon(height, extrudedHeight, 0, CesiumMath.EPSILON2);

        ellipseGeometry._center = ellipseGeometry._ellipsoid.scaleToGeodeticSurface(ellipseGeometry._center, ellipseGeometry._center);
        var options = {
            center : ellipseGeometry._center,
            semiMajorAxis : ellipseGeometry._semiMajorAxis,
            semiMinorAxis : ellipseGeometry._semiMinorAxis,
            ellipsoid : ellipseGeometry._ellipsoid,
            rotation : ellipseGeometry._rotation,
            height : height,
            granularity : ellipseGeometry._granularity,
            numberOfVerticalLines : ellipseGeometry._numberOfVerticalLines
        };
        var geometry;
        if (extrude) {
            options.extrudedHeight = extrudedHeight;
            options.offsetAttribute = ellipseGeometry._offsetAttribute;
            geometry = computeExtrudedEllipse(options);
        } else {
            geometry = computeEllipse(options);

            if (defined(ellipseGeometry._offsetAttribute)) {
                var length = geometry.attributes.position.values.length;
                var applyOffset = new Uint8Array(length / 3);
                var offsetValue = ellipseGeometry._offsetAttribute === GeometryOffsetAttribute.NONE ? 0 : 1;
                arrayFill(applyOffset, offsetValue);
                geometry.attributes.applyOffset = new GeometryAttribute({
                    componentDatatype : ComponentDatatype.UNSIGNED_BYTE,
                    componentsPerAttribute : 1,
                    values: applyOffset
                });
            }
        }

        return new Geometry({
            attributes : geometry.attributes,
            indices : geometry.indices,
            primitiveType : PrimitiveType.LINES,
            boundingSphere : geometry.boundingSphere,
            offsetAttribute : ellipseGeometry._offsetAttribute
        });
    };

    return EllipseOutlineGeometry;
});

define('Workers/createEllipseOutlineGeometry',[
        '../Core/Cartesian3',
        '../Core/defined',
        '../Core/EllipseOutlineGeometry',
        '../Core/Ellipsoid'
    ], function(
        Cartesian3,
        defined,
        EllipseOutlineGeometry,
        Ellipsoid) {
    'use strict';

    function createEllipseOutlineGeometry(ellipseGeometry, offset) {
        if (defined(offset)) {
            ellipseGeometry = EllipseOutlineGeometry.unpack(ellipseGeometry, offset);
        }
        ellipseGeometry._center = Cartesian3.clone(ellipseGeometry._center);
        ellipseGeometry._ellipsoid = Ellipsoid.clone(ellipseGeometry._ellipsoid);
        return EllipseOutlineGeometry.createGeometry(ellipseGeometry);
    }

    return createEllipseOutlineGeometry;
});

}());