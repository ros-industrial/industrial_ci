'use strict';
var {spawnSync} = require('child_process');
var env = Object.assign({}, process.env)
env.TARGET_REPO_PATH = env.GITHUB_WORKSPACE.slice()
env.TARGET_REPO_NAME = env.GITHUB_REPOSITORY.split('/').pop()
env._FOLDING_TYPE = 'github_actions'
var r = spawnSync(__dirname + '/../ci.sh', { stdio: 'inherit', env });
if (r.error) {
  throw r.error;
}
process.exit(r.status !== null ? r.status : 1);
