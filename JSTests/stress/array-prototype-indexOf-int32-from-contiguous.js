function shouldBe(a, b) {
    if (a !== b)
        throw new Error(`Expected ${b} but got ${a}`);
}

const array = [{}, {}, {}, 3.0, {}, {}, {}];

for (let i = 0; i < testLoopCount; i++)
    shouldBe(array.indexOf(3), 3);
