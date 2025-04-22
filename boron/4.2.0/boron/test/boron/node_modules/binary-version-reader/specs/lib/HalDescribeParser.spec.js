/*
 *  Copyright 2015 Particle ( https://particle.io )
 *
 *  Licensed under the Apache License, Version 2.0 (the "License");
 *  you may not use this file except in compliance with the License.
 *  You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 *  Unless required by applicable law or agreed to in writing, software
 *  distributed under the License is distributed on an "AS IS" BASIS,
 *  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *  See the License for the specific language governing permissions and
 *  limitations under the License.
 */

var fs = require('fs');
var expect = require('chai').expect;

var FirmwareModule = require('../../lib/FirmwareModule');
var HalDescribeParser = require('../../lib/HalDescribeParser');

describe('Describe parser', function() {
	describe('getModules', function(){
		it('returns null when describe is incorrect', function(){
			var parser = new HalDescribeParser();
			expect(parser.getModules()).be.null;
			expect(parser.getModules({})).be.null;
			expect(parser.getModules({m: 'foo'})).be.null;
		});

		it('returns FirmwareModules from describe', function(){
			var data = require('./../describes/fixed_dependencies_describe.json.js');
			var parser = new HalDescribeParser();
			var results = parser.getModules(data);
			expect(results).length(5);
			expect(results[0] instanceof FirmwareModule).be.true;
		});
	});

	describe('getSystemVersion', function(){
		it('returns null when describe is incorrect', function(){
			var parser = new HalDescribeParser();
			expect(parser.getSystemVersion()).be.null;
		});

		it('returns first system module version', function(){
			var data = require('./../describes/safe_mode_1.json.js');
			var parser = new HalDescribeParser();
			expect(parser.getSystemVersion(data)).to.equal(6);
		});

		it('returns system version for three modules', function(){
			var data = require('./../describes/three_system_modules.json.js');
			var parser = new HalDescribeParser();
			expect(parser.getSystemVersion(data)).to.equal(19);
		});

		it('works with three modules and firmware reset', function(){
			var data = require('./../describes/three_system_modules_with_firmware_reset.json.js');
			var parser = new HalDescribeParser();
			expect(parser.getSystemVersion(data)).to.equal(202);
			var modules = parser.getModules(data);
			var restoreModule = modules.filter(function(module){
				return module.isFactoryLocation();
			})[0];
			expect(restoreModule).to.not.be.undefined;
			expect(restoreModule.version).to.equal(3);
		});
	});
});
