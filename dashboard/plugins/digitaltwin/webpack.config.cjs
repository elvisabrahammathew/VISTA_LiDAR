const path = require('path');
const CopyWebpackPlugin = require('copy-webpack-plugin');
module.exports = {
  entry: './src/module.ts', target: 'web', devtool: 'source-map',
  output: {path: path.resolve(__dirname, 'dist'), filename: 'module.js', library: {type: 'amd'}, clean: true, publicPath: ''},
  externalsType: 'amd',
  externals: [({request}, callback) => request && (request === 'react' || request === 'react/jsx-runtime' || request.startsWith('@grafana/'))
    ? callback(null, request) : callback()],
  resolve: {extensions: ['.ts', '.tsx', '.js']},
  module: {rules: [{test: /\.tsx?$/, exclude: /node_modules/, use: {loader: 'ts-loader', options: {compilerOptions: {noEmit: false}}}}]},
  optimization: {splitChunks: false, runtimeChunk: false},
  plugins: [new CopyWebpackPlugin({patterns: [{from: 'src/plugin.json'}, {from: 'src/img', to: 'img'}, {from: 'README.md'}]})]
};
