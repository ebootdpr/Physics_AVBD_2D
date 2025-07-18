/*
* Copyright (c) 2025 Chris Giles
*
* Permission to use, copy, modify, distribute and sell this software
* and its documentation for any purpose is hereby granted without fee,
* provided that the above copyright notice appear in all copies.
* Chris Giles makes no representations about the suitability
* of this software for any purpose.
* It is provided "as is" without express or implied warranty.
*/

// static void (*scenes[])(Solver*) =
scenes: []Scene_Proc = .[
    sceneEmpty,
    sceneGround,
    sceneDynamicFriction,
    sceneStaticFriction,
    scenePyramid,
    sceneRope,
    sceneHeavyRope,
    sceneHangingRope,
    sceneSpring,
    sceneSpringsRatio,
    sceneStack,
    sceneStackRatio,
    sceneRod,
    sceneSoftBody,
    sceneJointGrid,
    sceneNet,
    sceneMotor,
    sceneFracture
];

sceneNames :: string.[
    "Empty",
    "Ground",
    "Dynamic Friction",
    "Static Friction",
    "Pyramid",
    "Rope",
    "Heavy Rope",
    "Hanging Rope",
    "Spring",
    "Spring Ratio",
    "Stack",
    "Stack Ratio",
    "Rod",
    "Soft Body",
    "Joint Grid",
    "Net",
    "Motor",
    "Fracture"
];

#no_reset sceneCount := #run sceneNames.count;
