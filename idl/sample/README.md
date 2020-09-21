# How to use these sample

## Convert JSON -> Flatbuffer binary

Run 
```
$ flatc -o idl/sample -b idl/outech.fbs idl/sample/pid_config.json
```

## Convert Flatbuffer binary -> JSON

```
$ flatc -o idl/sample --json --raw-binary idl/outech.fbs -- idl/sample/pid_config.bin
```
