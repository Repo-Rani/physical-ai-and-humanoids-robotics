
export default function Footer () {
    return (
        <div className='footer text-white pt-12 pb-8 px-6 md:px-16 lg:px-24 xl:px-32'>
            <div className='flex flex-wrap justify-between gap-8 md:gap-6'>
                <div className='max-w-80'>
                    <div className="text-2xl font-extrabold mb-4 footer__brand-text">
                        <span>
                            Humanoid Robotics
                        </span>
                    </div>
                    <p className='text-sm text-gray-300'>
                        Comprehensive textbook on Physical AI and Humanoid Robotics, covering ROS 2, simulation, and practical applications.
                    </p>
                    <div className='flex items-center gap-4 mt-6'>
                        {/* GitHub */}
                        <a href="#" className="text-gray-300 hover:text-green-400 transition-colors">
                            <svg className="w-6 h-6" fill="currentColor" viewBox="0 0 24 24">
                                <path d="M12 0c-6.626 0-12 5.373-12 12 0 5.302 3.438 9.8 8.207 11.387.599.111.793-.261.793-.577v-2.234c-3.338.726-4.033-1.416-4.033-1.416-.546-1.387-1.333-1.756-1.333-1.756-1.089-.745.083-.729.083-.729 1.205.084 1.839 1.237 1.839 1.237 1.07 1.834 2.807 1.304 3.492.997.107-.775.418-1.305.762-1.604-2.665-.305-5.467-1.334-5.467-5.931 0-1.311.469-2.381 1.236-3.221-.124-.303-.535-1.524.117-3.176 0 0 1.008-.322 3.301 1.23.957-.266 1.983-.399 3.003-.404 1.02.005 2.047.138 3.006.404 2.291-1.552 3.297-1.23 3.297-1.23.653 1.653.242 2.874.118 3.176.77.84 1.235 1.911 1.235 3.221 0 4.609-2.807 5.624-5.479 5.921.43.372.823 1.102.823 2.222v3.293c0 .319.192.694.801.576 4.765-1.589 8.199-6.086 8.199-11.386 0-6.627-5.373-12-12-12z"/>
                            </svg>
                        </a>
                        {/* Twitter */}
                        <a href="#" className="text-gray-300 hover:text-green-400 transition-colors">
                            <svg className="w-6 h-6" fill="currentColor" viewBox="0 0 24 24">
                                <path d="M22 5.92a8.2 8.2 0 01-2.36.65A4.1 4.1 0 0021.4 4a8.27 8.27 0 01-2.6 1A4.14 4.14 0 0016 4a4.15 4.15 0 00-4.15 4.15c0 .32.04.64.1.94a11.75 11.75 0 01-8.52-4.32 4.14 4.14 0 001.29 5.54A4.1 4.1 0 013 10v.05a4.15 4.15 0 003.33 4.07 4.12 4.12 0 01-1.87.07 4.16 4.16 0 003.88 2.89A8.33 8.33 0 012 19.56a11.72 11.72 0 006.29 1.84c7.55 0 11.68-6.25 11.68-11.67 0-.18 0-.35-.01-.53A8.18 8.18 0 0022 5.92z" />
                            </svg>
                        </a>
                        {/* LinkedIn */}
                        <a href="#" className="text-gray-300 hover:text-green-400 transition-colors">
                            <svg className="w-6 h-6" fill="currentColor" viewBox="0 0 24 24">
                                <path d="M4.98 3.5C3.88 3.5 3 4.38 3 5.48c0 1.1.88 1.98 1.98 1.98h.02c1.1 0 1.98-.88 1.98-1.98C6.98 4.38 6.1 3.5 4.98 3.5zM3 8.75h3.96V21H3V8.75zm6.25 0h3.8v1.68h.05c.53-.98 1.82-2.02 3.75-2.02 4.01 0 4.75 2.64 4.75 6.07V21H17v-5.63c0-1.34-.03-3.07-1.88-3.07-1.88 0-2.17 1.47-2.17 2.98V21H9.25V8.75z" />
                            </svg>
                        </a>
                    </div>
                </div>

                <div>
                    <p className='text-lg font-semibold text-white mb-4'>MODULES</p>
                    <ul className='mt-2 flex flex-col gap-2 text-sm'>
                        <li><a href="/docs/module-0-getting-started" className="text-gray-300 hover:text-green-400 transition-colors">Getting Started</a></li>
                        <li><a href="/docs/module-1-ros2" className="text-gray-300 hover:text-green-400 transition-colors">ROS 2 Fundamentals</a></li>
                        <li><a href="/docs/module-2-digital-twin" className="text-gray-300 hover:text-green-400 transition-colors">Digital Twin</a></li>
                        <li><a href="/docs/module-3-isaac" className="text-gray-300 hover:text-green-400 transition-colors">Isaac Sim</a></li>
                        <li><a href="/docs/module-4-vla" className="text-gray-300 hover:text-green-400 transition-colors">VLA & Manipulation</a></li>
                        <li><a href="/docs/module-5-capstone" className="text-gray-300 hover:text-green-400 transition-colors">Capstone Project</a></li>
                    </ul>
                </div>

                <div>
                    <p className='text-lg font-semibold text-white mb-4'>RESOURCES</p>
                    <ul className='mt-2 flex flex-col gap-2 text-sm'>
                        <li><a href="/docs/glossary" className="text-gray-300 hover:text-green-400 transition-colors">Glossary</a></li>
                        <li><a href="/docs/hardware-guide" className="text-gray-300 hover:text-green-400 transition-colors">Hardware Guide</a></li>
                        <li><a href="/docs/instructor-guide" className="text-gray-300 hover:text-green-400 transition-colors">Instructor Guide</a></li>
                        <li><a href="/docs/changelog" className="text-gray-300 hover:text-green-400 transition-colors">Changelog</a></li>
                        <li><a href="/docs/contributing" className="text-gray-300 hover:text-green-400 transition-colors">Contributing</a></li>
                    </ul>
                </div>

                <div className='max-w-80'>
                    <p className='text-lg font-semibold text-white mb-4'>NEWSLETTER</p>
                    <p className='mt-2 text-sm text-gray-300'>
                        Subscribe to receive updates on new content, robotics research, and educational resources.
                    </p>
                    <div className='flex items-center mt-4'>
                        <input type="email" className='bg-white text-gray-800 rounded-l-lg border border-gray-300 h-10 px-4 outline-none focus:ring-2 focus:ring-green-500 flex-grow' placeholder='Enter your email' />
                        <button className='flex items-center justify-center bg-green-600 hover:bg-green-700 h-10 w-12 rounded-r-lg transition-colors'>
                            <svg className="w-5 h-5 text-white" aria-hidden="true" xmlns="http://www.w3.org/2000/svg" width="24" height="24" fill="none" viewBox="0 0 24 24"><path stroke="currentColor" strokeLinecap="round" strokeLinejoin="round" strokeWidth="2" d="M19 12H5m14 0-4 4m4-4-4-4" /></svg>
                        </button>
                    </div>
                </div>
            </div>
            <hr className='border-gray-700 mt-10' />
            <div className='flex flex-col md:flex-row gap-4 items-center justify-between py-6'>
                <p className="text-gray-400">© {new Date().getFullYear()} AI & Humanoid Robotics Textbook. All rights reserved.</p>
                <ul className='flex items-center gap-6 text-sm'>
                    <li><a href="/docs/privacy" className="text-gray-400 hover:text-green-400 transition-colors">Privacy</a></li>
                    <li><a href="/docs/terms" className="text-gray-400 hover:text-green-400 transition-colors">Terms</a></li>
                    <li><a href="/docs/sitemap" className="text-gray-400 hover:text-green-400 transition-colors">Sitemap</a></li>
                </ul>
            </div>
        </div>
    );
};
