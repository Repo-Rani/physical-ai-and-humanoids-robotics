export default function Banner() {
    return (
        <a 
          href="http://localhost:3000/docs/intro" 
          className="flex items-center justify-center group w-full py-1.5 font-medium text-sm text-center bg-[var(--ifm-color-primary)] text-white"
        >
            <div className="flex flex-wrap items-center gap-1 justify-center group-hover:bg-white/15 transition duration-300 px-4 py-1 rounded-full hover:text-green-800">
                <p>Discover cutting-edge humanoid robotics revolutionizing human-machine interaction and automation.</p>
                <svg className="mt-px" width="16" height="16" viewBox="0 0 14 14" fill="none" xmlns="http://www.w3.org/2000/svg">
                    <path d="M2.91797 7H11.0846" stroke="currentColor" strokeWidth="1.5" strokeLinecap="round" strokeLinejoin="round"/>
                    <path d="M7 2.9165L11.0833 6.99984L7 11.0832" stroke="currentColor" strokeWidth="1.5" strokeLinecap="round" strokeLinejoin="round"/>
                </svg>
            </div>
        </a>
    );
}
