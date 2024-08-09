
"use strict";

let Detection3D = require('./Detection3D.js');
let Classification2D = require('./Classification2D.js');
let VisionInfo = require('./VisionInfo.js');
let Detection3DArray = require('./Detection3DArray.js');
let ObjectHypothesis = require('./ObjectHypothesis.js');
let Detection2DArray = require('./Detection2DArray.js');
let BoundingBox2D = require('./BoundingBox2D.js');
let BoundingBox2DArray = require('./BoundingBox2DArray.js');
let Detection2D = require('./Detection2D.js');
let BoundingBox3D = require('./BoundingBox3D.js');
let BoundingBox3DArray = require('./BoundingBox3DArray.js');
let Classification3D = require('./Classification3D.js');
let ObjectHypothesisWithPose = require('./ObjectHypothesisWithPose.js');

module.exports = {
  Detection3D: Detection3D,
  Classification2D: Classification2D,
  VisionInfo: VisionInfo,
  Detection3DArray: Detection3DArray,
  ObjectHypothesis: ObjectHypothesis,
  Detection2DArray: Detection2DArray,
  BoundingBox2D: BoundingBox2D,
  BoundingBox2DArray: BoundingBox2DArray,
  Detection2D: Detection2D,
  BoundingBox3D: BoundingBox3D,
  BoundingBox3DArray: BoundingBox3DArray,
  Classification3D: Classification3D,
  ObjectHypothesisWithPose: ObjectHypothesisWithPose,
};
