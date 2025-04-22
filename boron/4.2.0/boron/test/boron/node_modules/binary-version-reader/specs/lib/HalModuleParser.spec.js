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
 * Created by middleca on 6/11/15.
 */

var fs = require('fs');
var path = require('path');
var should = require('should');
var when = require('when');
var pipeline = require('when/pipeline');
var BufferOffset = require('buffer-offset');

var HalModuleParser = require('../../lib/HalModuleParser.js');
var ModuleInfo = require('../../lib/ModuleInfo.js');
var createFirmwareBinary = require('../../lib/firmwareTestHelper').createFirmwareBinary;
var config = require('../../lib/config.js').config;

var settings = {
	binaries: path.resolve(path.join(__dirname, '../binaries'))
};


describe('HalModuleParser', function () {
	it('should fail gracefully when the file doesn\'t exist or is empty', function (done) {
		var filename = path.join(settings.binaries, 'emptybin.bin');
		var parser = new HalModuleParser();
		parser._loadFile(filename)
			.then(
				done.bind(null, 'should have failed'),
				done.bind(null, null)
			);
	});

	it('should succeed when the file exists', function (done) {
		var filename = path.join(settings.binaries, '040_system-part1.bin');
		var parser = new HalModuleParser();
		parser._loadFile(filename)
			.then(
				done.bind(null, null),
				done.bind(null, 'should have passed')
			);
	});


	it('should validate the CRC in the binary', function (done) {
		var filename = path.join(settings.binaries, '040_system-part1.bin');
		var parser = new HalModuleParser();

		pipeline([
			function () {
				return parser._loadFile(filename);
			},
			function (buffer) {
				return parser._validateCRC(buffer);
			},
			function (crcInfo) {
				//console.log('got crcInfo ', crcInfo);

				should(crcInfo).be.ok;
				should(crcInfo.ok).be.ok;
				should(crcInfo.actualCrc).be.ok;
				should(crcInfo.storedCrc).eql(crcInfo.actualCrc);

				return when.resolve();
			}
		])
			.then(
				done.bind(null, null),
				done.bind(null));

	});

	it('should read prefix info from part 1', function (done) {
		var filename = path.join(settings.binaries, '040_system-part1.bin');
		var expectedPrefixInfo = {
			moduleStartAddy: '8020000',
			moduleEndAddy: '805cba4',
			reserved: 0,
			moduleFlags: ModuleInfo.Flags.NONE,
			moduleVersion: 1,
			platformID: 6,
			moduleFunction: 4,
			moduleIndex: 1,
			depModuleFunction: 0,
			depModuleIndex: 0,
			depModuleVersion: 0,
			dep2ModuleFunction: 0,
			dep2ModuleIndex: 0,
			dep2ModuleVersion: 0,
			prefixOffset: 388
		};

		var parser = new HalModuleParser();
		parser.parseFile(filename)
			.then(
				function (fileInfo) {
					//console.log('got part 1 info ', fileInfo.prefixInfo);

					should(fileInfo).be.ok;
					should(fileInfo.crc.ok).be.ok;
					should(fileInfo.prefixInfo).eql(expectedPrefixInfo);
					done();
				},
				function (err) {
					done(err)
				}).catch(done);
	});

	it('should read prefix info from part 2', function (done) {
		var filename = path.join(settings.binaries, '040_system-part2.bin');
		var expectedPrefixInfo = {
			moduleStartAddy: '8060000',
			moduleEndAddy: '807e954',
			reserved: 0,
			moduleFlags: ModuleInfo.Flags.NONE,
			moduleVersion: 1,
			platformID: 6,
			moduleFunction: 4,
			moduleIndex: 2,
			depModuleFunction: 4,
			depModuleIndex: 1,
			depModuleVersion: 1,
			dep2ModuleFunction: 0,
			dep2ModuleIndex: 0,
			dep2ModuleVersion: 0,
			prefixOffset: 388
		};

		var parser = new HalModuleParser();
		parser.parseFile(filename)
			.then(
				function (fileInfo) {
					//console.log('got part 2 info ', fileInfo.prefixInfo);
					should(fileInfo).be.ok;
					should(fileInfo.crc.ok).be.ok;
					should(fileInfo.prefixInfo).eql(expectedPrefixInfo);

					done();
				},
				function (err) {
					done(err)
				}).catch(done);
	});

	it('should read prefix info from a user module', function (done) {
		var filename = path.join(settings.binaries, '040_user-part.bin');
		var expectedPrefixInfo = {
			moduleStartAddy: '80a0000',
			moduleEndAddy: '80a128c',
			reserved: 0,
			moduleFlags: ModuleInfo.Flags.NONE,
			moduleVersion: 2,
			platformID: 6,
			moduleFunction: 5,
			moduleIndex: 1,
			depModuleFunction: 4,
			depModuleIndex: 2,
			depModuleVersion: 1,
			dep2ModuleFunction: 0,
			dep2ModuleIndex: 0,
			dep2ModuleVersion: 0,
			prefixOffset: 0
		};

		var parser = new HalModuleParser();
		parser.parseFile(filename)
			.then(
				function (fileInfo) {
					//console.log('got user info ', fileInfo.prefixInfo);
					should(fileInfo).be.ok;
					should(fileInfo.crc.ok).be.ok;
					should(fileInfo.prefixInfo).eql(expectedPrefixInfo);

					done();
				},
				function (err) {
					done(err)
				}).catch(done);
	});

	it('should read prefix info from monolithic firmware', function (done) {
		var filename = path.join(settings.binaries, '044_Core_Tinker_P5_V10.bin');
		var expectedPrefixInfo = {
			moduleStartAddy: '8005000',
			moduleEndAddy: '801a8e0',
			reserved: 0,
			moduleFlags: ModuleInfo.Flags.NONE,
			moduleVersion: 0,
			platformID: 0,
			moduleFunction: 3,
			moduleIndex: 0,
			depModuleFunction: 0,
			depModuleIndex: 0,
			depModuleVersion: 0,
			dep2ModuleFunction: 0,
			dep2ModuleIndex: 0,
			dep2ModuleVersion: 0,
			prefixOffset: 268
		};

		var parser = new HalModuleParser();
		parser.parseFile(filename)
			.then(
				function (fileInfo) {
					//console.log('got user info ', fileInfo.prefixInfo);
					should(fileInfo).be.ok;
					should(fileInfo.crc.ok).be.ok;
					should(fileInfo.prefixInfo).eql(expectedPrefixInfo);

					done();
				},
				function (err) {
					done(err)
				}).catch(done);
	});

	it('should read suffix info from system part 1', function (done) {
		var filename = path.join(settings.binaries, '040_system-part1.bin');
		var expectedSuffixInfo = {
			productId: -1,
			productVersion: -1,
			fwUniqueId: 'ecb6acb4cf75ca04169f2214a24c470516cabe91683ac3664bdd174c4bb50386',
			reserved: 0,
			suffixSize: 36,
			crcBlock: '5d7db471'
		};

		var parser = new HalModuleParser();
		parser.parseFile(filename)
			.then(
				function (fileInfo) {
					//console.log('got part 1 suffix ', fileInfo.suffixInfo);

					should(fileInfo).be.ok;
					should(fileInfo.crc.ok).be.ok;
					should(fileInfo.suffixInfo).eql(expectedSuffixInfo);
					done();
				},
				function (err) {
					done(err)
				}).catch(done);
	});

	it('should read suffix info from system part 2', function (done) {
		var filename = path.join(settings.binaries, '040_system-part2.bin');
		var expectedSuffixInfo = {
			productId: -1,
			productVersion: -1,
			fwUniqueId: 'ea3d9d175a6eee3d10023316240b51bea8200bdc182d0343801161b7ca53e2ae',
			reserved: 0,
			suffixSize: 36,
			crcBlock: '7db66023'
		};

		var parser = new HalModuleParser();
		parser.parseFile(filename)
			.then(
				function (fileInfo) {
					//console.log('got part 2 suffix ', fileInfo.suffixInfo);

					should(fileInfo).be.ok;
					should(fileInfo.crc.ok).be.ok;
					should(fileInfo.suffixInfo).eql(expectedSuffixInfo);
					done();
				},
				function (err) {
					done(err)
				}).catch(done);
	});


	it('should read suffix info from the user part', function (done) {
		var filename = path.join(settings.binaries, '040_user-part.bin');
		var expectedSuffixInfo = {
			productId: -1,
			productVersion: -1,
			fwUniqueId: 'f9f552aa98d7e3eab750862a01743024a4d05514021598a4341b3d83b37eda36',
			reserved: 0,
			suffixSize: 36,
			crcBlock: 'b138f375'
		};

		var parser = new HalModuleParser();
		parser.parseFile(filename)
			.then(
				function (fileInfo) {
					//console.log('got part 1 info ', fileInfo);

					should(fileInfo).be.ok;
					should(fileInfo.crc.ok).be.ok;
					should(fileInfo.suffixInfo).eql(expectedSuffixInfo);
					done();
				},
				function (err) {
					done(err)
				}).catch(done);
	});


	it('should read info from a bootloader module', function (done) {
		var filename = path.join(settings.binaries, 'RC4_bootloader_pad_BM-09.bin');

		var expectedPrefixInfo = {
			moduleStartAddy: '8000000',
			moduleEndAddy: '8003f98',
			reserved: 0,
			moduleFlags: ModuleInfo.Flags.NONE,
			moduleVersion: 2,
			platformID: 6,
			moduleFunction: 2,
			moduleIndex: 0,
			depModuleFunction: 0,
			depModuleIndex: 0,
			depModuleVersion: 0,
			dep2ModuleFunction: 0,
			dep2ModuleIndex: 0,
			dep2ModuleVersion: 0,
			prefixOffset: 388
		};

		var expectedSuffixInfo = {
			productId: 65535,
			productVersion: 65535,
			fwUniqueId: 'ffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffff',
			reserved: 65535,
			suffixSize: 65535,
			crcBlock: 'ffffffff'
		};

		var parser = new HalModuleParser();
		parser.parseFile(filename)
			.then(
				function (fileInfo) {
					//console.log('got bootloader info ', fileInfo);

					should(fileInfo).be.ok;

					//bootloader CRC is actually bad
					//should(fileInfo.crc.ok).be.ok;

					should(fileInfo.suffixInfo).eql(expectedSuffixInfo);
					should(fileInfo.prefixInfo).eql(expectedPrefixInfo);
					done();
				},
				function (err) {
					done(err)
				}).catch(done);
	});

	it('should have a working example', function (done) {
		var filename = path.join(settings.binaries, '040_user-part.bin');
		var Reader = require('../../main.js').HalModuleParser;
		var reader = new Reader();
		reader.parseFile(filename, function (fileInfo, err) {
			should(fileInfo).be.ok;
			done();
		});
	});

	it('should work with bluz system-part', function (done) {
		var filename = path.join(settings.binaries, '103_system-part1.bin');
		var expectedPrefixInfo = {
			moduleStartAddy: '18000',
			moduleEndAddy: '36768',
			reserved: 0,
			moduleFlags: ModuleInfo.Flags.NONE,
			moduleVersion: 1,
			platformID: 103,
			moduleFunction: 4,
			moduleIndex: 1,
			depModuleFunction: 0,
			depModuleIndex: 0,
			depModuleVersion: 0,
			dep2ModuleFunction: 0,
			dep2ModuleIndex: 0,
			dep2ModuleVersion: 0,
			prefixOffset: 192
		};

		var parser = new HalModuleParser();
		parser.parseFile(filename)
			.then(
				function (fileInfo) {
					//console.log('got part 2 info ', fileInfo.prefixInfo);
					should(fileInfo).be.ok;
					should(fileInfo.crc.ok).be.ok;
					should(fileInfo.prefixInfo).eql(expectedPrefixInfo);

					done();
				},
				function (err) {
					done(err)
				}).catch(done);
	});

	it('should work with xenon system part', function (done) {
		var filename = path.join(settings.binaries, '080_system-part1-xenon.bin');
		var expectedPrefixInfo = {
			moduleStartAddy: '30000',
			moduleEndAddy: 'c7580',
			reserved: 0,
			moduleFlags: ModuleInfo.Flags.NONE,
			moduleVersion: 312,
			platformID: 14,
			moduleFunction: 4,
			moduleIndex: 1,
			depModuleFunction: 0,
			depModuleIndex: 0,
			depModuleVersion: 0,
			dep2ModuleFunction: 2,
			dep2ModuleIndex: 0,
			dep2ModuleVersion: 101,
			prefixOffset: 512
		};

		var parser = new HalModuleParser();
		parser.parseFile(filename)
			.then(
				function (fileInfo) {
					should(fileInfo).be.ok;
					should(fileInfo.crc.ok).be.ok;
					should(fileInfo.prefixInfo).eql(expectedPrefixInfo);

					done();
				},
				function (err) {
					done(err)
				}).catch(done);
	});

	it('should work with xenon user part', function (done) {
		var filename = path.join(settings.binaries, '080_user-part-xenon.bin');
		var expectedPrefixInfo = {
			moduleStartAddy: 'd4000',
			moduleEndAddy: 'd4cec',
			reserved: 0,
			moduleFlags: ModuleInfo.Flags.NONE,
			moduleVersion: 5,
			platformID: 14,
			moduleFunction: 5,
			moduleIndex: 1,
			depModuleFunction: 4,
			depModuleIndex: 1,
			depModuleVersion: 312,
			dep2ModuleFunction: 0,
			dep2ModuleIndex: 0,
			dep2ModuleVersion: 0,
			prefixOffset: 0
		};

		var parser = new HalModuleParser();
		parser.parseFile(filename)
			.then(
				function (fileInfo) {
					should(fileInfo).be.ok;
					should(fileInfo.crc.ok).be.ok;
					should(fileInfo.prefixInfo).eql(expectedPrefixInfo);

					done();
				},
				function (err) {
					done(err)
				}).catch(done);
	});

	it('should work with xenon bootloader', function (done) {
		var filename = path.join(settings.binaries, '080_bootloader-xenon.bin');
		var expectedPrefixInfo = {
			moduleStartAddy: 'f4000',
			moduleEndAddy: 'fc164',
			reserved: 0,
			moduleFlags: ModuleInfo.Flags.NONE,
			moduleVersion: 211,
			platformID: 14,
			moduleFunction: 2,
			moduleIndex: 0,
			depModuleFunction: 0,
			depModuleIndex: 0,
			depModuleVersion: 0,
			dep2ModuleFunction: 0,
			dep2ModuleIndex: 0,
			dep2ModuleVersion: 0,
			prefixOffset: 512
		};

		var parser = new HalModuleParser();
		parser.parseFile(filename)
			.then(
				function (fileInfo) {
					should(fileInfo).be.ok;
					should(fileInfo.crc.ok).be.ok;
					should(fileInfo.prefixInfo).eql(expectedPrefixInfo);

					done();
				},
				function (err) {
					done(err)
				}).catch(done);
	});

	it('should work with argon ncp', function (done) {
		var filename = path.join(settings.binaries, 'argon-ncp-firmware-0.0.5-ota.bin');
		var expectedPrefixInfo = {
			moduleStartAddy: '0',
			moduleEndAddy: 'ca73c',
			reserved: 33,
			moduleFlags: ModuleInfo.Flags.NONE,
			moduleVersion: 5,
			platformID: 12,
			moduleFunction: 7,
			moduleIndex: 0,
			depModuleFunction: 0,
			depModuleIndex: 0,
			depModuleVersion: 0,
			dep2ModuleFunction: 0,
			dep2ModuleIndex: 0,
			dep2ModuleVersion: 0,
			prefixOffset: 0
		};

		var parser = new HalModuleParser();
		parser.parseFile(filename)
			.then(
				function (fileInfo) {
					should(fileInfo).be.ok;
					should(fileInfo.crc.ok).be.ok;
					should(fileInfo.prefixInfo).eql(expectedPrefixInfo);

					done();
				},
				function (err) {
					done(err)
				}).catch(done);
	});

	it('should work with boron-som system part', function (done) {
		var filename = path.join(settings.binaries, 'boron-som-system-part1@1.1.0-rc.1.bin');
		var expectedPrefixInfo = {
			moduleStartAddy: '30000',
			moduleEndAddy: 'ce668',
			reserved: 0,
			moduleFlags: ModuleInfo.Flags.NONE,
			moduleVersion: 1100,
			platformID: 23,
			moduleFunction: 4,
			moduleIndex: 1,
			depModuleFunction: 2,
			depModuleIndex: 0,
			depModuleVersion: 201,
			dep2ModuleFunction: 0,
			dep2ModuleIndex: 0,
			dep2ModuleVersion: 0,
			prefixOffset: 512
		};

		var parser = new HalModuleParser();
		parser.parseFile(filename)
			.then(
				function (fileInfo) {
					should(fileInfo).be.ok;
					should(fileInfo.crc.ok).be.ok;
					should(fileInfo.prefixInfo).eql(expectedPrefixInfo);

					done();
				},
				function (err) {
					done(err)
				}).catch(done);
	});

	it('should work with argon softdevice (radio stack)', function (done) {
		var filename = path.join(settings.binaries, 'argon-softdevice-6.1.1.bin');
		var expectedPrefixInfo = {
			moduleStartAddy: '1000',
			moduleEndAddy: '25e24',
			reserved: 0,
			moduleFlags: ModuleInfo.Flags.DROP_MODULE_INFO,
			moduleVersion: 182,
			platformID: ModuleInfo.Platform.ARGON,
			moduleFunction: ModuleInfo.FunctionType.RADIO_STACK,
			moduleIndex: 0,
			depModuleFunction: ModuleInfo.FunctionType.SYSTEM_PART,
			depModuleIndex: 1,
			depModuleVersion: 1300,
			dep2ModuleFunction: ModuleInfo.FunctionType.BOOTLOADER,
			dep2ModuleIndex: 0,
			dep2ModuleVersion: 311,
			prefixOffset: 0
		};

		var parser = new HalModuleParser();
		parser.parseFile(filename)
			.then(
				function (fileInfo) {
					should(fileInfo).be.ok;
					should(fileInfo.crc.ok).be.ok;
					should(fileInfo.prefixInfo).eql(expectedPrefixInfo);

					done();
				},
				function (err) {
					done(err)
				}).catch(done);
	});

	describe('given a module descriptor', function () {
		function buildModule(module) {
			var buffer = BufferOffset.convert(Buffer.alloc(24));
			return appendModule(buffer, module);
		}

		function appendModule(buffer, module) {
			buffer.appendUInt32LE(module.moduleStartAddy);
			buffer.appendUInt32LE(module.moduleEndAddy);
			buffer.appendUInt16LE(0);       // reserved
			buffer.appendUInt16LE(module.moduleVersion);
			buffer.appendUInt16LE(module.platformID);
			buffer.appendUInt8(module.moduleFunction);
			buffer.appendUInt8(module.moduleIndex);
			appendModuleDependency(buffer, module.depModuleFunction, module.depModuleIndex, module.depModuleVersion);
			appendModuleDependency(buffer, module.dep2ModuleFunction, module.dep2ModuleIndex, module.dep2ModuleVersion);
			return buffer;
		}

		function appendModuleDependency(buffer, f, n, v) {
			buffer.appendUInt8(f);
			buffer.appendUInt8(n);
			buffer.appendUInt16LE(v);
			return buffer;
		}

		var buffer;

		var testModule1 = {
			moduleStartAddy: 0x12345678,
			moduleEndAddy: 0x87654321,
			moduleFlags: ModuleInfo.Flags.NONE,
			moduleVersion: 1234,
			platformID: 4567,
			moduleFunction: 147,
			moduleIndex: 139,
			depModuleFunction: 250,
			depModuleIndex: 251,
			depModuleVersion: 345,
			dep2ModuleFunction: 252,
			dep2ModuleIndex: 252,
			dep2ModuleVersion: 3456

		};

		beforeEach(function () {
			buffer = buildModule(testModule1);
		});

		describe('when the buffer is parsed', function () {
			var module;
			beforeEach(function () {
				var parser = new HalModuleParser();
				var buf = Buffer.alloc(buffer.length);
				buffer.copy(buf);
				parser._findModulePrefixOffset = function () {
					return 0;
				};
				module = parser._readPrefix(buffer);
			});

			it('parses the module version as 16-bits LE', function () {
				should(module).have.property('depModuleVersion').eql(345);
			});

			it('parses the 2nd module version as 16-bits LE', function () {
				should(module).have.property('dep2ModuleVersion').eql(3456);
			});

		});

	});

	it('allows parsing prefix and suffix info separately', function () {
		var file = path.join(settings.binaries, '040_system-part1.bin');
		var expectedPrefixInfo = {
			moduleStartAddy: '8020000',
			moduleEndAddy: '805cba4',
			reserved: 0,
			moduleFlags: ModuleInfo.Flags.NONE,
			moduleVersion: 1,
			platformID: 6,
			moduleFunction: 4,
			moduleIndex: 1,
			depModuleFunction: 0,
			depModuleIndex: 0,
			depModuleVersion: 0,
			dep2ModuleFunction: 0,
			dep2ModuleIndex: 0,
			dep2ModuleVersion: 0,
			prefixOffset: 388
		};
		var expectedSuffixInfo = {
			productId: -1,
			productVersion: -1,
			fwUniqueId: 'ecb6acb4cf75ca04169f2214a24c470516cabe91683ac3664bdd174c4bb50386',
			reserved: 0,
			suffixSize: 36,
			crcBlock: '5d7db471'
		};
		var fileBuffer = fs.readFileSync(file);
		var parser = new HalModuleParser();
		return parser.parsePrefix({ fileBuffer: fileBuffer })
			.then(function (fileInfo) {
				should(fileInfo.prefixInfo).eql(expectedPrefixInfo);
				should(fileInfo).not.have.property('suffixInfo');
				return parser.parseSuffix({ fileBuffer: fileBuffer });
			})
			.then(function (fileInfo) {
				should(fileInfo.suffixInfo).eql(expectedSuffixInfo);
				should(fileInfo).not.have.property('prefixInfo');
			})
	});

	it('parses extended product id', function () {
		var filename = path.join(settings.binaries, 'extended-product-id.bin');

		var parser = new HalModuleParser();
		return parser.parseFile(filename)
			.then(function (fileInfo) {
				should(fileInfo.suffixInfo.productId).eql(0xaabbccdd);
			});
	});

	describe('given that a custom CRC-32 function is provided globally', function () {
		var defaultCrc32 = null;

		before(function () {
			defaultCrc32 = config().crc32;
		});

		afterEach(function () {
			config({ crc32: defaultCrc32 });
		});

		it('uses that function for CRC-32 computations', function () {
			var dummyCrc = Buffer.from([0xaa, 0xbb, 0xcc, 0xdd]);
			config({
				crc32: function (buf) {
					return dummyCrc;
				}
			});
			var parser = new HalModuleParser();
			return parser.parseBuffer({ fileBuffer: createFirmwareBinary() })
				.then(function (fileInfo) {
					should(fileInfo.crc.actualCrc).eql(dummyCrc.toString('hex'));
				});
		});
	});

	it('should work with unknown valid module', function (done) {
		const filename = path.join(settings.binaries, 'unknown.bin');
		const expectedPrefixInfo = {
			moduleStartAddy: '20003000',
			moduleEndAddy: '2000503c',
			reserved: 129,
			moduleFlags: 128,
			moduleVersion: 9999,
			platformID: 129,
			moduleFunction: 129,
			moduleIndex: 99,
			depModuleFunction: 129,
			depModuleIndex: 99,
			depModuleVersion: 999,
			dep2ModuleFunction: 129,
			dep2ModuleIndex: 29,
			dep2ModuleVersion: 129,
			prefixOffset: 16
		};

		const parser = new HalModuleParser();
		parser.parseFile(filename)
			.then(
				function (fileInfo) {
					should(fileInfo).be.ok;
					should(fileInfo.crc.ok).be.ok;
					should(fileInfo.prefixInfo).eql(expectedPrefixInfo);

					done();
				},
				function (err) {
					done(err)
				}).catch(done);
	});
});
