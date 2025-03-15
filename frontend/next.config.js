/** @type {import('next').NextConfig} */
const nextConfig = {
  // Ignore ESLint errors during build
  eslint: {
    // Warning instead of error during build
    ignoreDuringBuilds: true,
  },
  // Ignore TypeScript errors during build
  typescript: {
    // Warning instead of error during build
    ignoreBuildErrors: true,
  },
}

module.exports = nextConfig
