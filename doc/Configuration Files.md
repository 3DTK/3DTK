# Configuration Files

## .ini configuration

Applies to

- show
- QtShow
- wxShow.

The program's options are read from

1. a file in the user's application settings folder,
2. a file in the dataset directory, and then
3. the command line.

They take precedence in that order, i.e. a value given on the command line overwrites any previous value of that option.

You can view available options by calling the program with `--help`.

Example: `bin/show --help`

### Config file format

- The program expects one `option name`, `option value` pair separated by `=` per line.
- Spaces around the `=` are allowed.
- Only the long option names are valid.
- Flag options must be given as `true` or `false` to set them.
- Comments are started with `#`.

Example:

```ini
format = uos_rrgbt
end = 10 # only 11 scans in this dataset
advanced = true
```

### Per-user configuration

A per-user configuration file is useful to customize `show` to your system and your needs.

An example of useful options:

```ini
advanced = true # always display the advanced GUI
dimensions = 1600x1000 # bigger initial window, if your window manager doesn't manage it
invertmousex = true
invertmousey = true # FPS camera movement
fov = 90 # Field of view
fps = 144 # 144hz screen
no-fog = true
scanserver = true # Always use scanserver
```

#### Windows

The per-user configuration file is located at `%APPDATA%\3dtk\show.ini`.

#### Others (Linux, Mac OS)

The per-user configuration file is located at `$XDG_CONFIG_HOME/3dtk/show.ini`, which defaults to `$HOME/.config/3dtk/show.ini`.

### Per-dataset configuration

A per-dataset configuration file is useful to record dataset properties. You will not have to specify them on the command line and can share them along with the dataset.

It is read from `config.ini` in the dataset directory.

Example: You invoke `bin/show dat`. `show` looks for `dat/config.ini`.

Useful options:

```ini
format = riegl_txt # Scan format. You should always set this.
end = 12 # index of the last scan file
scale = 1 # scale unit size
min = 1
max = 254 # omit scan artifacts
```

## Per-dataset `format` file

Applies to

- floorplan
- model
- scan_diff
- veloslam.

These programs can read a file named `format` in the dataset directory.
They **only** support the options `start`, `end` and `format`.
The file format is the same as above, but there must be no comments.

Example:

```ini
format = xyz
start = 2
end = 10
```
