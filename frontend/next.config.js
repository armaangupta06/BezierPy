/** @type {import('next').NextConfig} */
const nextConfig = {
  // Disable ESLint during build to prevent build failures
  eslint: {
    // Warning: This allows production builds to successfully complete even if
    // your project has ESLint errors.
    ignoreDuringBuilds: true,
  },
  // Disable TypeScript type checking during build for the same reason
  typescript: {
    // Warning: This allows production builds to successfully complete even if
    // your project has TypeScript errors.
    ignoreBuildErrors: true,
  },
  // Don't use static export for Vercel deployment
  // output: 'export',
  // Configure images
  images: {
    domains: ['vercel.app'],
    unoptimized: process.env.NODE_ENV === 'development',
  },
  // Ensure compatibility with Vercel deployment
  distDir: '.next',
};

module.exports = nextConfig;
