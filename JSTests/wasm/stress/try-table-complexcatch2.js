
import { instantiate } from "../wabt-wrapper.js"
import * as assert from "../assert.js"

/*
(module
  (tag $e0)
  (tag $e1)
  (tag $e2)
  (func (export "catch_complex_2") (param i32) (result i32)
    (block $h0
      (block $h1
        (try_table (result i32) (catch $e0 $h0) (catch $e1 $h1)
          (if (i32.eqz (local.get 0))
            (then (throw $e0))
            (else
              (if (i32.eq (local.get 0) (i32.const 1))
                (then (throw $e1))
                (else (throw $e2))
              )
            )
           )
          (i32.const 2)
        )
        (return)
      )
      (return (i32.const 4))
    )
    (i32.const 3)
  )
)
*/

var wasm_module = new WebAssembly.Module(new Uint8Array([
0x00,0x61,0x73,0x6d,0x01,0x00,0x00,0x00,0x01,0x89,0x80,0x80,0x80,0x00,0x02,0x60,0x00,0x00,0x60,0x01,0x7f,0x01,0x7f,0x03,0x82,0x80,0x80,0x80,0x00,0x01,0x01,0x0d,0x87,0x80,0x80,0x80,0x00,0x03,0x00,0x00,0x00,0x00,0x00,0x00,0x07,0x93,0x80,0x80,0x80,0x00,0x01,0x0f,0x63,0x61,0x74,0x63,0x68,0x5f,0x63,0x6f,0x6d,0x70,0x6c,0x65,0x78,0x5f,0x32,0x00,0x00,0x0a,0xb6,0x80,0x80,0x80,0x00,0x01,0xb0,0x80,0x80,0x80,0x00,0x00,0x02,0x40,0x02,0x40,0x1f,0x7f,0x02,0x00,0x00,0x01,0x00,0x01,0x00,0x20,0x00,0x45,0x04,0x40,0x08,0x00,0x05,0x20,0x00,0x41,0x01,0x46,0x04,0x40,0x08,0x01,0x05,0x08,0x02,0x0b,0x0b,0x41,0x02,0x0b,0x0f,0x0b,0x41,0x04,0x0f,0x0b,0x41,0x03,0x0b,
]));

var instance = new WebAssembly.Instance(wasm_module);

const { catch_complex_2 } = instance.exports

for (let i = 0; i < wasmTestLoopCount; ++i) {
    assert.eq(catch_complex_2(0), 3)
    assert.eq(catch_complex_2(1), 4)
    try {
        catch_complex_2(2)
        print("no")
    } catch (e) {
    }
}
