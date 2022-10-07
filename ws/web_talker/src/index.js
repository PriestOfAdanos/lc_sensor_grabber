const http = require('http')
const fs = require('fs')

const server = http.createServer((req, res) => {
  if (req.url == '/') {
    res.writeHead(200, { 'content-type': 'text/html' })
    fs.createReadStream('/home/lc/ws/web_talker/src/index.html').pipe(res)
  }
  if (req.url == '/scripts/vue_frontend.js') {
    res.writeHead(200, { 'content-type': 'text/javascript' })
    fs.createReadStream(__dirname+'/frontend_dist/vue_frontend.js').pipe(res)
  }
  if (req.url == '/styles/styles.css') {
    res.writeHead(200, { 'content-type': 'text/css' })
    fs.createReadStream(__dirname+'/styles/styles.css').pipe(res)
  }
})
server.listen(process.env.PORT || 3000)