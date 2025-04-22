'use strict';
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

/**
 * Created by middleca on 6/18/15.
 */

var fs = require('fs');
var path = require('path');
var should = require('should');
var extend = require('xtend');
var when = require('when');
var pipeline = require('when/pipeline');
var HalDependencyResolver = require ('../../lib/HalDependencyResolver.js');
var FirmwareModule = require('../../lib/FirmwareModule');
var expect = require('chai').expect;

var settings = {
	binaries: path.resolve(path.join(__dirname, '../binaries'))
};


describe('HalDependencyResolver', function() {


	it('repairs errors in module names and versions', function() {
		var photonReportedModules = require('./../describes/describe.json.js').m;
		var fixedTestData = require('./../describes/fixed_describe.json.js').m;

		var resolver = new HalDependencyResolver();
		var result = resolver._repairDescribeErrors(photonReportedModules);

		should(result).eql(fixedTestData);
	});

	it('converts binary metadata into describe format', function() {
		var userPrefixInfo = {
			depModuleFunction: 4,
			depModuleIndex: 2,
			depModuleVersion: 1
		};

		var expectedDescribe = {
			f: 's',
			n: '2',
			v: 1
		};

		var resolver = new HalDependencyResolver();
		var result = resolver._binaryDepsToDescribe(userPrefixInfo);

		should(result).eql(expectedDescribe);
	});


	it('walk a dependency chain nicely', function() {
		var fixedTestData = require('./../describes/fixed_describe.json.js').m;
		var safeBinaryReqs = {
			f: 's',
			n: '2',
			v: 1
		};

		var resolver = new HalDependencyResolver();
		var arr = resolver._walkChain(fixedTestData, safeBinaryReqs);
		should(arr).eql([]);
	});

	it('flag a module for update when out of version', function() {
		var fixedTestData = require('./../describes/fixed_describe.json.js').m;
		var systemPart2 = fixedTestData[2];

		var safeBinaryReqs = {
			f: 's',
			n: '2',
			v: systemPart2.v + 1
		};

		var shouldBeMissing = extend(fixedTestData[2], { v: systemPart2.v + 1, d: [] });

		var resolver = new HalDependencyResolver();
		var arr = resolver._walkChain(fixedTestData, safeBinaryReqs);
		should(arr.length).eql(1);
		should(arr[0]).eql(shouldBeMissing);
	});


	/**
	 * loads the system parts,
	 * reads their binary info,
	 *
	 * passes in a user binary
	 * reads its binary info / dependencies
	 *
	 * walks recursively up the dependency chain
	 *
	 * matches dependencies to our known good modules,
	 *
	 * AND RETURNS THEM.
	 *
	 *
		 ______     __     __     ______     ______     ______
		/\  ___\   /\ \  _ \ \   /\  ___\   /\  ___\   /\__  _\
		\ \___  \  \ \ \/ ".\ \  \ \  __\   \ \  __\   \/_/\ \/
		 \/\_____\  \ \__/".~\_\  \ \_____\  \ \_____\    \ \_\
		  \/_____/   \/_/   \/_/   \/_____/   \/_____/     \/_/

	 */
	it('passes a full test', function(done) {

		var resolver = new HalDependencyResolver();

		var part1 = path.join(settings.binaries, '../binaries/040_system-part1.bin');
		var part2 = path.join(settings.binaries, '../binaries/040_system-part2.bin');

		// load those modules in!
		resolver.assimilateModule(part1);
		resolver.assimilateModule(part2);


		var describeFilename = path.join(settings.binaries, '../describes/old_describe.json.js');
		var oldDescribe = require(describeFilename).m;

		var userFirmware = path.join(settings.binaries, '../binaries/040_user-part.bin');
		var fileBuffer = fs.readFileSync(userFirmware);

		//
		// given a describe message from a device, and some user firmware, get the modules we need to run it.
		//
		var result = resolver.parseAndResolve(oldDescribe, fileBuffer)
			.then(function(result) {
				should(result).be.ok;
				console.log('dependency resolve result had ', result.length, ' items ');
				should(result.length).eql(2);

				//the first thing should be part1, and the second thing part2

				should(result[0].filename).endWith('system-part1.bin');
				should(result[1].filename).endWith('system-part2.bin');

				done();
			}, function(err) {
				done(err);
			}).catch(function(err) {
				done(err);
			});
	});

	it('recommend modules when going from 042 to 043', function(done) {

		var resolver = new HalDependencyResolver();

		var part1 = path.join(settings.binaries, '../binaries/043_system-part1.bin');
		var part2 = path.join(settings.binaries, '../binaries/043_system-part2.bin');

		// load those modules in!
		resolver.assimilateModule(part1);
		resolver.assimilateModule(part2);


		var describeFilename = path.join(settings.binaries, '../describes/042_describe.json.js');
		var oldDescribe = require(describeFilename).m;

		var userFirmware = path.join(settings.binaries, '../binaries/043_user-part.bin');
		var fileBuffer = fs.readFileSync(userFirmware);

		//
		// given a describe message from a device, and some user firmware, get the modules we need to run it.
		//
		var result = resolver.parseAndResolve(oldDescribe, fileBuffer)
			.then(function(result) {
				should(result).be.ok;
				console.log('dependency resolve result had ', result.length, ' items ');
				should(result.length).eql(2);

				//the first thing should be part1, and the second thing part2

				should(result[0].filename).endWith('system-part1.bin');
				should(result[1].filename).endWith('system-part2.bin');

				done();
			}, function(err) {
				done(err);
			}).catch(function(err) {
				done(err);
			});
	});

	it('rejects when detects missing dependencies', function(done){
		var userModuleSafeMode = require('./../describes/safe_mode_2.json.js');
		var resolver = new HalDependencyResolver();

		var deps = resolver.findAnyMissingDependencies(userModuleSafeMode);
		should(deps).be.ok;
		should(deps.length).eql(1);
		should(deps[0].f).eql('s');
		should(deps[0].n).eql('2');
		should(deps[0].v).eql(8);

		resolver.userModuleHasMissingDependencies(userModuleSafeMode)
			.then(
			function(result) {
				done(new Error("Should have rejected: " + result));
			},
			function(err) {
				should(err).be.ok;
				should(err.length).eql(1);

				// show me the module that needs replacing!
				err[0].func.should.eql('s');
				err[0].name.should.eql('2');
				err[0].version.should.eql(8);
				done();
			});
	});

	it('resolves when dependencies are met', function(done){
		var data = require('./../describes/fixed_dependencies_describe.json.js');
		var resolver = new HalDependencyResolver();


		var deps = resolver.findAnyMissingDependencies(data);
		should(deps).be.ok;
		should(deps.length).eql(0);

		resolver.userModuleHasMissingDependencies(data)
			.then(function(result) {
				done();
			}, function(err) {
				done(err || "Should not have rejected");
			});
	});

	it('solves firmware module', function(){
		var data = require('./../describes/fixed_dependencies_describe.json.js');
		var resolver = new HalDependencyResolver();
		var module = new FirmwareModule({
			d: [
				{
					f: 's',
					n: '2',
					v: 2
				}
			]
		});

		var result = resolver.solveFirmwareModule(data.m, module.dependencies[0]);
		result.should.eql([]);

		var requiredVersion = 5;
		module.dependencies[0].version = requiredVersion;
		result = resolver.solveFirmwareModule(data.m, module.dependencies[0]);
		result.length.should.eql(1);
		result[0].v.should.eql(requiredVersion);
	});

	it('finds missing dependencies for example 1', function(done) {
		var data = require('./../describes/safe_mode_1.json.js');
		var resolver = new HalDependencyResolver();
		var results = resolver.findAnyMissingDependencies(data);

		should(results).be.ok;
		should(results.length).be.greaterThan(0);

		var shouldBeMissing = data.m[2].d[0];
		var dep = results[0];


		//[ { s: 262144, l: 'm', vc: 30, vv: 30, f: 's', n: '1', v: 6, d: [] } ]
		should(dep.f).eql(shouldBeMissing.f);
		should(dep.n).eql(shouldBeMissing.n);
		should(dep.v).eql(shouldBeMissing.v);

		done();
	});


	/**
	 * userModuleHasMissingDependencies should resolve if "monolithic describe" (missing user module)
	 */
	it('handles missing user module appropriately', function(done) {

		var describe = require('./../describes/describe_no_usermodule.js');
		var resolver = new HalDependencyResolver();


		// should have nothing missing
		var deps = resolver.findAnyMissingDependencies(describe);
		should(deps).be.ok;
		should(deps.length).eql(0);

		// should resolve
		resolver.userModuleHasMissingDependencies(describe)
			.then(
			function(result) {
				done();
			},
			function(err) {
				done(new Error("Should have resolved: " + err));
			});
	});

	/**
	 * userModuleHasMissingDependencies should resolve if "monolithic describe" (missing user module)
	 */
	it('handles missing user module with missing stuff appropriately', function(done) {

		var describe = require('./../describes/describe_no_usermodule2.js');
		var resolver = new HalDependencyResolver();


		// should have nothing missing
		var deps = resolver.findAnyMissingDependencies(describe);
		should(deps).be.ok;
		should(deps.length).eql(1);

		should(deps[0].f).eql('s');
		should(deps[0].n).eql('1');
		should(deps[0].v).eql(3);

		// should resolve without finding anything, because this describe has no usermodule.
		resolver.userModuleHasMissingDependencies(describe)
			.then(
			function(result) {
				done();
			},
			function(err) {
				done(new Error("Should have resolved: " + err));
			});
	});

	it('handles weird double-safe-mode case', function(done) {

		var describe = require('./../describes/safe_mode_3.json.js');
		var resolver = new HalDependencyResolver();


		// should have nothing missing
		var deps = resolver.findAnyMissingDependencies(describe);
		should(deps).be.ok;
		should(deps.length).eql(2);

		// since we're processing the modules in listed order, and since I think
		// they're also sorta in dependency order (part1 comes before part2), I think we're okay here.
		// there is a small risk of getting into a scenario where we keep trying to update part2,
		// instead of first sending part1.

		// so when picking an update in the case of multiple dependencies, we should pick the thing that
		// itself has no dependencies if available.

		done();


	});

	it('handles multiple dependencies', function(done) {

		var describe = require('./../describes/multiple_dependencies.json.js');
		var resolver = new HalDependencyResolver();


		var deps = resolver.findAnyMissingDependencies(describe);
		should(deps).be.ok;
		should(deps.length).eql(1);

		// The missing dependency is a bootloader
		var m = describe.m[2].d[1];
		should(deps[0].f).eql(m.f);
		should(deps[0].n).eql(m.n);
		should(deps[0].v).eql(m.v);

		done();
	});


	describe('factory image', () => {
		function hasMissingDependencies(describe) {
			const modules = new HalDependencyResolver().findAnyMissingDependencies(describe);
			return modules && modules.length > 0;
		}

		it('does not report safe mode for factory modules that are invalid', () => {

			const describe = { 'p':13,'m':[{ 's':49152,'l':'m','vc':30,'vv':30,'f':'b','n':'0','v':300,'d':[] },{ 's':671744,'l':'m','vc':30,'vv':30,'f':'s','n':'1','v':403,'d':[{ 'f':'b','n':'0','v':101,'_':'' },{ 'f':'n','n':'0','v':0,'_':'' }] },{ 's':131072,'l':'m','vc':30,'vv':30,'u':'09B7930F35B990D0021501E62007AE48BBC68E6422CE4504B2BA6DEA7F5E1C45','f':'u','n':'1','v':5,'d':[{ 'f':'s','n':'1','v':403,'_':'' },{ 'f':'n','n':'0','v':0,'_':'' }] },{ 's':131072,'l':'f','vc':30,'vv':0,'d':[{ 'f':'_','n':'6','v':0,'_':'' },{ 'f':'_','n':'6','v':0,'_':'' }] }] };

			expect(hasMissingDependencies(describe)).to.be.eql(false);
		});

		it('does not report safe mode for factory modules that are invalid', () => {

			const describe = { 'p':13,'m':[{ 's':49152,'l':'m','vc':30,'vv':30,'f':'b','n':'0','v':300,'d':[] },{ 's':671744,'l':'m','vc':30,'vv':30,'f':'s','n':'1','v':403,'d':[{ 'f':'b','n':'0','v':101,'_':'' },{ 'f':'n','n':'0','v':0,'_':'' }] },{ 's':131072,'l':'m','vc':30,'vv':30,'u':'09B7930F35B990D0021501E62007AE48BBC68E6422CE4504B2BA6DEA7F5E1C45','f':'u','n':'1','v':5,'d':[{ 'f':'s','n':'1','v':403,'_':'' },{ 'f':'n','n':'0','v':0,'_':'' }] },{ 's':131072,'l':'f','vc':30,'vv':0,'d':[{ 'f':'_','n':'6','v':0,'_':'' },{ 'f':'_','n':'6','v':0,'_':'' }] }] };

			expect(hasMissingDependencies(describe)).to.be.eql(false);
		});

		it('does not report safe mode for factory modules that are valid', () => {

			const describe = { 'p':13,'m':[{ 's':49152,'l':'m','vc':30,'vv':30,'f':'b','n':'0','v':300,'d':[] },{ 's':671744,'l':'m','vc':30,'vv':30,'f':'s','n':'1','v':403,'d':[{ 'f':'b','n':'0','v':101,'_':'' },{ 'f':'n','n':'0','v':0,'_':'' }] },{ 's':131072,'l':'m','vc':30,'vv':30,'u':'09B7930F35B990D0021501E62007AE48BBC68E6422CE4504B2BA6DEA7F5E1C45','f':'u','n':'1','v':5,'d':[{ 'f':'s','n':'1','v':403,'_':'' },{ 'f':'n','n':'0','v':0,'_':'' }] },{ 's':131072,'l':'f','vc':30,'vv':30,'d':[{ 'f':'_','n':'6','v':0,'_':'' },{ 'f':'_','n':'6','v':0,'_':'' }] }] };

			expect(hasMissingDependencies(describe)).to.be.eql(false);
		});


		it('does not report safe mode for no missing dependencies', () => {

			const describe = { 'p':13,'m':[{ 's':49152,'l':'m','vc':30,'vv':30,'f':'b','n':'0','v':300,'d':[] },{ 's':671744,'l':'m','vc':30,'vv':30,'f':'s','n':'1','v':403,'d':[{ 'f':'b','n':'0','v':101,'_':'' },{ 'f':'n','n':'0','v':0,'_':'' }] },{ 's':131072,'l':'m','vc':30,'vv':30,'u':'09B7930F35B990D0021501E62007AE48BBC68E6422CE4504B2BA6DEA7F5E1C45','f':'u','n':'1','v':5,'d':[{ 'f':'s','n':'1','v':403,'_':'' },{ 'f':'n','n':'0','v':0,'_':'' }] }] };

			expect(hasMissingDependencies(describe)).to.be.eql(false);
		});

		it('reports safe mode for missing application dependencies', () => {

			const describe = { 'p':13,'m':[{ 's':49152,'l':'m','vc':30,'vv':30,'f':'b','n':'0','v':300,'d':[] },{ 's':671744,'l':'m','vc':30,'vv':30,'f':'s','n':'1','v':403,'d':[{ 'f':'b','n':'0','v':101,'_':'' },{ 'f':'n','n':'0','v':0,'_':'' }] },{ 's':131072,'l':'m','vc':30,'vv':30,'u':'09B7930F35B990D0021501E62007AE48BBC68E6422CE4504B2BA6DEA7F5E1C45','f':'u','n':'1','v':5,'d':[{ 'f':'s','n':'1','v':9999,'_':'' },{ 'f':'n','n':'0','v':0,'_':'' }] }] };

			expect(hasMissingDependencies(describe)).to.be.eql(true);
		});

	});

});
