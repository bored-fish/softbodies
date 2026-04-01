const canvas = document.getElementById('canvas');
const ctx = canvas.getContext('2d');

canvas.width = window.innerWidth;
canvas.height = window.innerHeight;

ctx.fillStyle = '#1a1a2e';
ctx.fillRect(0, 0, canvas.width, canvas.height);

ctx.fillStyle = '#ff6b6b';
ctx.beginPath();
ctx.arc(canvas.width / 2, canvas.height / 2, 6, 0, Math.PI * 2);
ctx.fill();
