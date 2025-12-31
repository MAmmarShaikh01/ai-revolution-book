import { themes as prismThemes } from 'prism-react-renderer';
import type { Config } from '@docusaurus/types';
import type * as Preset from '@docusaurus/preset-classic';

// Physical AI & Humanoid Robotics Textbook configuration
const config: Config = {
  title: 'Physical AI & Humanoid Robotics',
  tagline: 'Embodied Intelligence for Next-Generation Robots',
  favicon: 'img/favicon.ico',

  url: 'https://ai-revolution-book.vercel.app', // for Vercel deployment
  baseUrl: '/', // Root base URL for Vercel

  organizationName: 'MAmmarShaikh01', // Your GitHub username
  projectName: 'ai-revolution-book', // Your repository name

  onBrokenLinks: 'throw',
  onBrokenMarkdownLinks: 'warn',
  markdown: {
    mermaid: true,
    mdx1Compat: {
      comments: true,
      admonitions: true,
      headingIds: true,
    },
  },

  i18n: {
    defaultLocale: 'en',
    locales: ['en'], // You can add 'ur' here if you plan to support Urdu translation
  },

  presets: [
    [
      'classic',
      {
        docs: {
          sidebarPath: './sidebars.ts', // Path to your sidebar file
          editUrl: 'https://github.com/MAmmarShaikh01/ai-revolution-book/tree/main/website/',
        },
        blog: false, // Disable blog (if you don't need it)
        theme: {
          customCss: './src/css/custom.css', // Path to custom styles (if any)
        },
      } satisfies Preset.Options,
    ],
  ],

  plugins: [
    [
      '@docusaurus/plugin-content-docs',
      {
        id: 'docs-software',
        path: 'docs-software',
        routeBasePath: 'docs-software', // Custom route
        sidebarPath: './sidebars.ts',
        editUrl: 'https://github.com/MAmmarShaikh01/ai-revolution-book/tree/main/website/',
      },
    ],
    [
      '@docusaurus/plugin-content-docs',
      {
        id: 'docs-hardware',
        path: 'docs-hardware',
        routeBasePath: 'docs-hardware', // Custom route
        sidebarPath: './sidebars.ts',
        editUrl: 'https://github.com/MAmmarShaikh01/ai-revolution-book/tree/main/website/',
      },
    ],
    [
      '@docusaurus/plugin-content-docs',
      {
        id: 'docs-urdu',
        path: 'docs-urdu',
        routeBasePath: 'docs-urdu', // Custom route
        sidebarPath: './sidebars-urdu.ts', // Ensure this file exists
        editUrl: 'https://github.com/MAmmarShaikh01/ai-revolution-book/tree/main/website/',
      },
    ],
    [
      '@docusaurus/plugin-content-docs',
      {
        id: 'docs-urdu-software',
        path: 'docs-urdu-software',
        routeBasePath: 'docs-urdu-software', // Custom route
        sidebarPath: './sidebars-urdu.ts', // Ensure this file exists
        editUrl: 'https://github.com/MAmmarShaikh01/ai-revolution-book/tree/main/website/',
      },
    ],
    [
      '@docusaurus/plugin-content-docs',
      {
        id: 'docs-urdu-hardware',
        path: 'docs-urdu-hardware',
        routeBasePath: 'docs-urdu-hardware', // Custom route
        sidebarPath: './sidebars-urdu.ts', // Ensure this file exists
        editUrl: 'https://github.com/MAmmarShaikh01/ai-revolution-book/tree/main/website/',
      },
    ],
  ],

  themeConfig: {
    image: 'img/docusaurus-social-card.jpg',
    navbar: {
      title: 'Physical AI',
      logo: {
        alt: 'Physical AI Logo',
        src: 'img/logo.svg',
      },
      items: [
        {
          type: 'docSidebar',
          sidebarId: 'tutorialSidebar', // Ensure this sidebar ID exists
          position: 'left',
          label: 'Textbook',
        },
        {
          href: 'https://github.com/MAmmarShaikh01/ai-revolution-book',
          label: 'GitHub',
          position: 'right',
        },
      ],
    },
    footer: {
      style: 'dark',
      links: [
        {
          title: 'Textbook',
          items: [
            {
              label: 'Introduction',
              to: '/docs/intro',
            },
            {
              label: 'Get Started',
              to: '/docs/module-1/physical-ai-foundations',
            },
          ],
        },
        {
          title: 'Resources',
          items: [
            {
              label: 'GitHub',
              href: 'https://github.com/MAmmarShaikh01/ai-revolution-book',
            },
            {
              label: 'Panaversity',
              href: 'https://panaversity.org',
            },
          ],
        },
      ],
      copyright: `Copyright © ${new Date().getFullYear()} Physical AI & Humanoid Robotics Textbook. All rights reserved.`,
    },
    prism: {
      theme: prismThemes.github,
      darkTheme: prismThemes.dracula,
      additionalLanguages: ['python', 'bash', 'json', 'yaml', 'cpp'],
    },
  } satisfies Preset.ThemeConfig,
};

export default config;
