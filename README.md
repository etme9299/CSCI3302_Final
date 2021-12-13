# CSCI3302_Final
Final Project for CSCI 3302 - Fall 2021

## Installation

Although release-it is a **generic** release tool, installation requires npm. To use release-it, a `package.json` file
is not required. The recommended way to install release-it also adds basic configuration. Answer one or two questions
and it's ready:

```bash
npm init release-it
```

Alternatively, install it manually, and add the `release` script to `package.json`:

```bash
npm install --save-dev release-it
```

```json
{
  "name": "my-package",
  "version": "1.0.0",
  "scripts": {
    "release": "release-it"
  },
  "devDependencies": {
    "release-it": "*"
  }
}
```

Now you can run `npm run release` from the command line (any release-it arguments behind the `--`):

```bash
npm run release
npm run release -- minor --ci
```
