const path = require('node:path');
const CopyWebpackPlugin = require('copy-webpack-plugin');
module.exports = {
  entry: './src/module.ts', target: 'web', devtool: 'source-map',
  output: {path: path.resolve(__dirname, 'dist'), filename: 'module.js', library: {type: 'amd'}, clean: true, publicPath: ''},
  externalsType: 'amd',
  externals: [/^@grafana\//, 'react', 'react-dom', 'react/jsx-runtime', 'rxjs', 'rxjs/operators'],
  resolve: {extensions: ['.ts', '.js']},
  module: {rules: [{test: /\.ts$/, exclude: /node_modules/, use: {loader: 'ts-loader', options: {compilerOptions: {noEmit: false}}}}]},
  optimization: {splitChunks: false, runtimeChunk: false},
  plugins: [new CopyWebpackPlugin({patterns: [{from: 'src/plugin.json', to: 'plugin.json'}, {from: 'src/img', to: 'img'}, {from: 'INSTALL.md', to: 'INSTALL.md'}]})],
};
