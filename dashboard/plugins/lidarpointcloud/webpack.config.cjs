const path = require('path');
const CopyWebpackPlugin = require('copy-webpack-plugin');

module.exports = (_env, argv) => ({
  mode: argv.mode || 'production',
  entry: './src/module.ts',
  target: 'web',
  devtool: argv.mode === 'production' ? 'source-map' : 'eval-source-map',
  output: {
    path: path.resolve(__dirname, 'dist'),
    filename: 'module.js',
    library: { type: 'amd' },
    clean: true,
    publicPath: '',
  },
  externalsType: 'amd',
  externals: [
    ({ request }, callback) => {
      if (
        request &&
        (request === 'react' ||
          request === 'react-dom' ||
          request === 'react/jsx-runtime' ||
          request.startsWith('@grafana/'))
      ) {
        return callback(null, request);
      }
      return callback();
    },
  ],
  resolve: {
    extensions: ['.ts', '.tsx', '.js'],
  },
  module: {
    rules: [
      {
        test: /\.tsx?$/,
        exclude: /node_modules/,
        use: {
          loader: 'ts-loader',
          options: {
            transpileOnly: false,
            compilerOptions: { noEmit: false },
          },
        },
      },
    ],
  },
  optimization: {
    splitChunks: false,
    runtimeChunk: false,
  },
  plugins: [
    new CopyWebpackPlugin({
      patterns: [
        { from: 'src/plugin.json', to: 'plugin.json' },
        { from: 'src/img', to: 'img' },
        { from: 'README.md', to: 'README.md' },
        { from: 'CHANGELOG.md', to: 'CHANGELOG.md' },
        { from: 'INSTALL.md', to: 'INSTALL.md' },
        { from: 'LICENSE', to: 'LICENSE.txt' },
      ],
    }),
  ],
});
