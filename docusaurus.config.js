// @ts-check
import { themes as prismThemes } from 'prism-react-renderer';

/** @type {import('@docusaurus/types').Config} */
const config = {
  title: 'Physical AI & Humanoid Robotics',
  tagline: 'From Simulation to Reality',
  favicon: 'img/favicon.ico',

  url: 'https://your-github-username.github.io',
  baseUrl: '/ai-and-humanoid-robotics-hackathon/',

  organizationName: 'your-organization',
  projectName: 'ai-and-humanoid-robotics-hackathon',

  onBrokenLinks: 'throw',
  onBrokenMarkdownLinks: 'warn',

  presets: [
    [
      'classic',
      {
        docs: {
          sidebarPath: require.resolve('./sidebars.js'),
          editUrl:
            'https://github.com/your-repo/ai-and-humanoid-robotics-hackathon/edit/main/',
          showLastUpdateTime: true,
        },
        blog: false,

        theme: {
          customCss: require.resolve('./src/css/custom.css'),
        },
      },
    ],
  ],

  markdown: {
    mermaid: true,
  },

  themes: ['@docusaurus/theme-mermaid'],

  themeConfig: {
    image: 'img/docusaurus-social-card.jpg',

    /** 🔥 IMPORTANT: SEARCH COMPLETELY DISABLED */
    algolia: null,

    navbar: {
      title: 'Physical AI Textbook',
      items: [
        {
          type: 'doc',
          docId: 'intro',
          position: 'left',
          label: 'Textbook',
        },
        {
          href: 'https://github.com/your-repo/ai-and-humanoid-robotics-hackathon',
          label: 'GitHub',
          position: 'right',
        },
      ],
    },

    prism: {
      theme: prismThemes.github,
      darkTheme: prismThemes.dracula,
      additionalLanguages: ['python', 'bash', 'yaml', 'cpp'],
    },
  },
};

export default config;
