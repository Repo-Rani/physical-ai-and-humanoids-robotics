// @ts-check

/** @type {import('@docusaurus/plugin-content-docs').SidebarsConfig} */
const sidebars = {
  textbookSidebar: [
    {
      type: 'doc',
      id: 'intro',
      label: '🏠 Home',
    },
    {
      type: 'category',
      label: 'Module 0: Getting Started (Weeks 1-2)',
      collapsible: true,
      collapsed: false,
      items: [
        'module-0-getting-started/module-0-chapter-1',
        'module-0-getting-started/module-0-chapter-2',
        'module-0-getting-started/module-0-chapter-3',
      ],
    },
    {
      type: 'category',
      label: 'Module 1: ROS 2 Fundamentals (Weeks 3-5)',
      collapsible: true,
      collapsed: true,
      items: [
        'module-1-ros2/module-1-chapter-1',
        'module-1-ros2/module-1-chapter-2',
        'module-1-ros2/module-1-chapter-3',
        'module-1-ros2/module-1-chapter-4',
        'module-1-ros2/module-1-chapter-5',
        'module-1-ros2/module-1-chapter-6',
      ],
    },
    {
      type: 'category',
      label: 'Module 2: Digital Twin (Weeks 6-7)',
      collapsible: true,
      collapsed: true,
      items: [
        'module-2-digital-twin/module-2-chapter-1',
        'module-2-digital-twin/module-2-chapter-2',
        'module-2-digital-twin/module-2-chapter-3',
        'module-2-digital-twin/module-2-chapter-4',
        'module-2-digital-twin/module-2-chapter-5',
        'module-2-digital-twin/module-2-chapter-6',
      ],
    },
    {
      type: 'category',
      label: 'Module 3: NVIDIA Isaac (Weeks 8-10)',
      collapsible: true,
      collapsed: true,
      items: [
        'module-3-isaac/module-3-chapter-1',
        'module-3-isaac/module-3-chapter-2',
        'module-3-isaac/module-3-chapter-3',
        'module-3-isaac/module-3-chapter-4',
        'module-3-isaac/module-3-chapter-5',
        'module-3-isaac/module-3-chapter-6',
      ],
    },
    {
      type: 'category',
      label: 'Module 4: VLA Models (Weeks 11-12)',
      collapsible: true,
      collapsed: true,
      items: [
        'module-4-vla/module-4-chapter-1',
        'module-4-vla/module-4-chapter-2',
        'module-4-vla/module-4-chapter-3',
        'module-4-vla/module-4-chapter-4',
        'module-4-vla/module-4-chapter-5',
      ],
    },
    {
      type: 'category',
      label: 'Module 5: Capstone Project (Weeks 13-14)',
      collapsible: true,
      collapsed: true,
      items: [
        'module-5-capstone/module-5-chapter-1',
        'module-5-capstone/module-5-chapter-2',
        'module-5-capstone/module-5-chapter-3',
        'module-5-capstone/module-5-chapter-4',
        'module-5-capstone/module-5-chapter-5',
        'module-5-capstone/module-5-chapter-6',
      ],
    },
    {
      type: 'category',
      label: '📚 Resources',
      collapsible: true,
      collapsed: true,
      items: [
        'hardware-guide',
        'glossary',
        'instructor-guide',
        'changelog',
      ],
    },
  ],
};

export default sidebars;
